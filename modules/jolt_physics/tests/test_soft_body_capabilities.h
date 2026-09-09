/**************************************************************************/
/*  test_soft_body_capabilities.h                                         */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#pragma once

#include "soft_body_cap_probe.h"

#include "core/config/project_settings.h"
#include "core/object/class_db.h"
#include "core/variant/typed_array.h"
#include "servers/physics_3d/physics_server_3d.h"
#include "servers/physics_3d/physics_server_3d_dummy.h"
#include "servers/physics_3d/physics_server_3d_extension.h"
#include "servers/rendering/rendering_server.h"

#include "tests/test_macros.h"

namespace TestJoltSoftBodyCap {

namespace Probe = JoltSoftBodyCapProbe;

constexpr const char *K_JOINTS = "rod/joints";
constexpr const char *K_COMPLIANCE = "rod/compliance";
constexpr const char *K_BEND = "rod/bend";
constexpr const char *K_FIXED = "rod/fixed";
constexpr const char *K_STATE = "rod/state";

// `PhysicsServer3D::get_singleton()` is null for every case in this file --
// `tests/test_main.cpp` brings physics up only for names containing `[SceneTree]`
// or `[Editor]`. So the backend is obtained explicitly, and the singleton the
// constructor silently overwrote is put back on the way out even if a `REQUIRE`
// aborts the case body.
//
// One server per NAME is built for the whole file and then reused, never
// finished: a Godot process only ever brings one physics backend up and down, and
// cycling `init()`/`finish()` per case corrupts memory. Measured over 12 suite
// runs with a fresh server per case, a different case aborted or segfaulted each
// time -- `Job::completed_head` (fixed in `jolt_job_system.cpp`) was only the
// visible half; Jolt's `Factory`/`RegisterTypes` globals are torn down and rebuilt
// by every `finish()`/`init()` pair too. Per-case isolation comes from the space
// and the bodies, which every case still creates and frees.
class ServerScope {
	PhysicsServer3D *saved = nullptr;
	PhysicsServer3D *server = nullptr;

public:
	explicit ServerScope(const String &p_name) {
		saved = PhysicsServer3D::get_singleton();

		static HashMap<String, PhysicsServer3D *> servers;
		if (HashMap<String, PhysicsServer3D *>::Iterator found = servers.find(p_name)) {
			server = found->value;
		} else {
			server = PhysicsServer3DManager::get_singleton()->new_server(p_name);
			if (server != nullptr) {
				server->init();
				server->set_active(true);
			}
			servers.insert(p_name, server);
		}

		// Reuse does not run the constructor that claimed the singleton.
		if (server != nullptr) {
			PhysicsServer3D::set_singleton_for_tests(server);
		}
	}

	~ServerScope() {
		PhysicsServer3D::set_singleton_for_tests(saved);
	}

	bool is_valid() const { return server != nullptr; }
	PhysicsServer3D *ptr() const { return server; }
	PhysicsServer3D *operator->() const { return server; }
};

// Collects diagnostics so a row can assert that an error names the offending
// numbers, rather than only that a call returned `false`. Printing stays off so
// the deliberate failures do not drown the suite's output; error handlers run
// regardless of that flag.
class ErrorCapture {
	static void _handle(void *p_userdata, const char *p_function, const char *p_file, int p_line, const char *p_error, const char *p_message, bool p_editor_notify, ErrorHandlerType p_type) {
		Vector<String> *messages = static_cast<Vector<String> *>(p_userdata);
		messages->push_back(String::utf8(p_error) + " " + String::utf8(p_message));
	}

	ErrorHandlerList list;
	Vector<String> messages;

public:
	ErrorCapture() {
		list.errfunc = &_handle;
		list.userdata = &messages;
		add_error_handler(&list);
		ERR_PRINT_OFF;
	}

	~ErrorCapture() {
		ERR_PRINT_ON;
		remove_error_handler(&list);
	}

	void clear() { messages.clear(); }
	int count() const { return messages.size(); }

	bool has(const String &p_fragment) const {
		for (const String &message : messages) {
			if (message.contains(p_fragment)) {
				return true;
			}
		}
		return false;
	}

	String joined() const {
		String out;
		for (const String &message : messages) {
			out += message + "\n";
		}
		return out;
	}
};

// A space that can be stepped. Order matches `Main::iteration`.
class SpaceScope {
	PhysicsServer3D *server = nullptr;
	RID space;

public:
	explicit SpaceScope(PhysicsServer3D *p_server) :
			server(p_server) {
		space = server->space_create();
		server->space_set_active(space, true);
	}

	~SpaceScope() { server->free_rid(space); }

	RID rid() const { return space; }

	void step(int p_frames = 1) {
		for (int i = 0; i < p_frames; i++) {
			server->sync();
			server->flush_queries();
			server->end_sync();
			server->step(1.0 / 60.0);
		}
	}
};

// A straight chain along +X with unit spacing, so every derived segment length
// is exactly 1 and the Bishop frames are all the same rotation.
PackedVector3Array make_joints(int p_count, real_t p_spacing = 1.0) {
	PackedVector3Array joints;
	for (int i = 0; i < p_count; i++) {
		joints.push_back(Vector3(p_spacing * i, 0, 0));
	}
	return joints;
}

PackedFloat32Array make_floats(int p_count, float p_value = 0.0f) {
	PackedFloat32Array values;
	for (int i = 0; i < p_count; i++) {
		values.push_back(p_value);
	}
	return values;
}

// The four writable keys for a `p_joints`-joint rod, all valid.
bool set_valid_rod(PhysicsServer3D *p_server, RID p_body, int p_joints, int p_fixed = 1) {
	bool ok = p_server->soft_body_set_extra_property(p_body, K_JOINTS, make_joints(p_joints));
	ok = p_server->soft_body_set_extra_property(p_body, K_COMPLIANCE, make_floats(p_joints - 1)) && ok;
	ok = p_server->soft_body_set_extra_property(p_body, K_BEND, make_floats(MAX(p_joints - 2, 0))) && ok;
	ok = p_server->soft_body_set_extra_property(p_body, K_FIXED, p_fixed) && ok;
	return ok;
}

// Two triangles sharing an edge: the smallest thing the cloth path accepts.
RID make_quad_mesh() {
	RenderingServer *rendering = RenderingServer::get_singleton();

	PackedVector3Array vertices;
	vertices.push_back(Vector3(0, 0, 0));
	vertices.push_back(Vector3(1, 0, 0));
	vertices.push_back(Vector3(1, 0, 1));
	vertices.push_back(Vector3(0, 0, 1));

	PackedInt32Array indices;
	const int triangles[6] = { 0, 1, 2, 0, 2, 3 };
	for (int index : triangles) {
		indices.push_back(index);
	}

	Array arrays;
	arrays.resize(RSE::ARRAY_MAX);
	arrays[RSE::ARRAY_VERTEX] = vertices;
	arrays[RSE::ARRAY_INDEX] = indices;

	RID mesh = rendering->mesh_create();
	rendering->mesh_add_surface_from_arrays(mesh, RSE::PRIMITIVE_TRIANGLES, arrays);
	return mesh;
}

Dictionary find_property(const TypedArray<Dictionary> &p_list, const String &p_name) {
	for (int i = 0; i < p_list.size(); i++) {
		const Dictionary entry = p_list[i];
		if (String(entry["name"]) == p_name) {
			return entry;
		}
	}
	return Dictionary();
}

// The live rotation of rod `p_index`, read out of `rod/state`.
Quaternion read_rod_rotation(PhysicsServer3D *p_server, RID p_body, int p_index) {
	const PackedFloat32Array state = p_server->soft_body_get_extra_property(p_body, K_STATE);
	if (state.size() < (p_index + 1) * 4) {
		return Quaternion(0, 0, 0, 0);
	}
	return Quaternion(state[p_index * 4 + 0], state[p_index * 4 + 1], state[p_index * 4 + 2], state[p_index * 4 + 3]);
}

TEST_SUITE("[JoltSoftBodyCap]") {

/* ---------------------------------------------------------------- protocol */

TEST_CASE("unknown prefix is silent") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();

	ErrorCapture errors;
	CHECK_FALSE(server->soft_body_set_extra_property(body, "nope/x", 1));
	CHECK(errors.count() == 0);

	server->free_rid(body);
}

TEST_CASE("misspelled key errors") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();

	ErrorCapture errors;
	CHECK_FALSE(server->soft_body_set_extra_property(body, "rod/joint", make_joints(9)));
	CHECK(errors.has("rod/joint"));
	CHECK(errors.has("not a valid key"));

	server->free_rid(body);
}

TEST_CASE("wrong type errors") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();

	ErrorCapture errors;
	CHECK_FALSE(server->soft_body_set_extra_property(body, K_JOINTS, 5));
	CHECK(errors.has("PackedVector3Array"));
	CHECK(errors.has("int"));

	server->free_rid(body);
}

TEST_CASE("get of unknown prefix is empty") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();

	CHECK(server->soft_body_get_extra_property(body, "nope/x").get_type() == Variant::NIL);

	server->free_rid(body);
}

TEST_CASE("jolt lists rod properties") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();

	const TypedArray<Dictionary> list = server->soft_body_get_extra_property_list(body);
	CHECK(list.size() >= 5);
	for (const char *key : { K_JOINTS, K_COMPLIANCE, K_BEND, K_FIXED, K_STATE }) {
		CHECK_MESSAGE(!find_property(list, key).is_empty(), key);
	}

	server->free_rid(body);
}

TEST_CASE("godot physics lists nothing") {
	ServerScope server("GodotPhysics3D");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();

	CHECK(server->soft_body_get_extra_property_list(body).size() == 0);

	server->free_rid(body);
}

TEST_CASE("rod state is not serialised") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();

	const Dictionary entry = find_property(server->soft_body_get_extra_property_list(body), K_STATE);
	REQUIRE_FALSE(entry.is_empty());
	CHECK(((uint32_t)entry["usage"] & PROPERTY_USAGE_STORAGE) == 0);

	server->free_rid(body);
}

TEST_CASE("rod joints are serialised") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();

	const Dictionary entry = find_property(server->soft_body_get_extra_property_list(body), K_JOINTS);
	REQUIRE_FALSE(entry.is_empty());
	CHECK(((uint32_t)entry["usage"] & PROPERTY_USAGE_STORAGE) != 0);

	server->free_rid(body);
}

TEST_CASE("wrapper forwards all three") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());

	// Every Jolt server is wrapped, so there is no unwrapped result to compare
	// against; what can be shown is that the server the test holds is not the
	// backend, and that all three answers still arrive through it.
	CHECK_FALSE(Probe::is_inner_jolt_server(server.ptr()));

	RID body = server->soft_body_create();
	CHECK(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(9)));
	CHECK(PackedVector3Array(server->soft_body_get_extra_property(body, K_JOINTS)).size() == 9);
	CHECK(server->soft_body_get_extra_property_list(body).size() >= 5);

	server->free_rid(body);
}

TEST_CASE("threaded wrapper forwards") {
	ProjectSettings *settings = ProjectSettings::get_singleton();
	const String key = "physics/3d/run_on_separate_thread";
	const Variant saved = settings->has_setting(key) ? settings->get_setting(key) : Variant(false);
	settings->set_setting(key, true);

	{
		// The flag is read when the server is constructed, not when it steps.
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());

		RID body = server->soft_body_create();
		CHECK(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(9)));
		CHECK(PackedVector3Array(server->soft_body_get_extra_property(body, K_JOINTS)).size() == 9);
		CHECK(server->soft_body_get_extra_property_list(body).size() >= 5);
		server->free_rid(body);
	}

	settings->set_setting(key, saved);
}

TEST_CASE("writable keys read back") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();

	const PackedVector3Array joints = make_joints(9);
	REQUIRE(server->soft_body_set_extra_property(body, K_JOINTS, joints));

	const PackedVector3Array read = server->soft_body_get_extra_property(body, K_JOINTS);
	REQUIRE(read.size() == joints.size());
	for (int i = 0; i < joints.size(); i++) {
		CHECK(read[i] == joints[i]);
	}

	server->free_rid(body);
}

TEST_CASE("rod state cannot be written") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();

	ErrorCapture errors;
	CHECK_FALSE(server->soft_body_set_extra_property(body, K_STATE, make_floats(32)));
	CHECK(errors.has("read-only"));

	server->free_rid(body);
}

TEST_CASE("invalid rid is rejected") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());

	ErrorCapture errors;
	SUBCASE("set") {
		CHECK_FALSE(server->soft_body_set_extra_property(RID(), K_JOINTS, make_joints(9)));
	}
	SUBCASE("get") {
		CHECK(server->soft_body_get_extra_property(RID(), K_JOINTS).get_type() == Variant::NIL);
	}
	SUBCASE("list") {
		CHECK(server->soft_body_get_extra_property_list(RID()).size() == 0);
	}
	CHECK(errors.count() > 0);
}

TEST_CASE("freed rid is rejected") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());

	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9));
	server->free_rid(body);

	ErrorCapture errors;
	SUBCASE("set") {
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(9)));
	}
	SUBCASE("get") {
		CHECK(server->soft_body_get_extra_property(body, K_JOINTS).get_type() == Variant::NIL);
	}
	SUBCASE("list") {
		CHECK(server->soft_body_get_extra_property_list(body).size() == 0);
	}
	CHECK(errors.count() > 0);
}

TEST_CASE("rod state needs a space") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9));

	ErrorCapture errors;
	CHECK(server->soft_body_get_extra_property(body, K_STATE).get_type() == Variant::NIL);
	CHECK(errors.count() > 0);

	server->free_rid(body);
}

TEST_CASE("writable keys read without a space") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9));

	ErrorCapture errors;
	// `live_read == false` skips the in-space guard, or a serialiser would save
	// nothing for a body that is not in a space yet.
	CHECK(PackedVector3Array(server->soft_body_get_extra_property(body, K_JOINTS)).size() == 9);
	CHECK(errors.count() == 0);

	server->free_rid(body);
}

TEST_CASE("rod state is valid before stepping") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());

	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9));
	server->soft_body_set_space(body, space.rid());
	REQUIRE(Probe::in_space(body));

	const PackedFloat32Array state = server->soft_body_get_extra_property(body, K_STATE);
	REQUIRE(state.size() == 8 * 4);
	for (int i = 0; i < state.size(); i++) {
		CHECK(Math::is_finite(state[i]));
	}

	server->free_rid(body);
}

TEST_CASE("godot physics ignores rod keys") {
	ServerScope server("GodotPhysics3D");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();

	ErrorCapture errors;
	CHECK_FALSE(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(9)));
	CHECK(errors.count() == 0);

	server->free_rid(body);
}

TEST_CASE("extension shims forward") {
	// A `PhysicsServer3DExtension` has no C++ implementation to call; what the
	// shim contributes is the bound virtual, which is what a GDExtension
	// overrides. Its absence is the one silent omission in this phase.
	// The shim itself is callable in every build. With nothing overriding the
	// virtual, `EXBIND*R`'s required call reports one error per method and hands
	// back the return type's default -- so three errors and three defaults prove
	// all three C++ overrides exist and reach their virtual.
	PhysicsServer3D *saved = PhysicsServer3D::get_singleton();
	PhysicsServer3DExtension *server = memnew(PhysicsServer3DExtension);
	const RID body;
	{
		ErrorCapture errors;
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(9)));
		CHECK(server->soft_body_get_extra_property(body, K_JOINTS).get_type() == Variant::NIL);
		CHECK(server->soft_body_get_extra_property_list(body).is_empty());
		CHECK(errors.count() == 3);
	}
	memdelete(server);
	PhysicsServer3D::set_singleton_for_tests(saved);

	// `ClassDB` only records virtual methods under `DEBUG_ENABLED`
	// (`class_db.cpp`: both `add_virtual_method` and `get_virtual_methods` are
	// wholly inside that guard), so the argument counts a GDExtension binds
	// against can only be read back from a debug build.
#ifdef DEBUG_ENABLED
	List<MethodInfo> methods;
	ClassDB::get_virtual_methods("PhysicsServer3DExtension", &methods);

	HashMap<String, int> arg_counts;
	for (const MethodInfo &method : methods) {
		arg_counts[method.name] = method.arguments.size();
	}

	REQUIRE(arg_counts.has("_soft_body_set_extra_property"));
	CHECK(arg_counts["_soft_body_set_extra_property"] == 3);
	REQUIRE(arg_counts.has("_soft_body_get_extra_property"));
	CHECK(arg_counts["_soft_body_get_extra_property"] == 2);
	REQUIRE(arg_counts.has("_soft_body_get_extra_property_list"));
	CHECK(arg_counts["_soft_body_get_extra_property_list"] == 1);
#endif
}

TEST_CASE("dummy server answers emptily") {
	PhysicsServer3D *saved = PhysicsServer3D::get_singleton();
	PhysicsServer3DDummy *server = memnew(PhysicsServer3DDummy);

	RID body = server->soft_body_create();
	CHECK_FALSE(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(9)));
	CHECK(server->soft_body_get_extra_property(body, K_JOINTS).get_type() == Variant::NIL);
	CHECK(server->soft_body_get_extra_property_list(body).size() == 0);

	memdelete(server);
	PhysicsServer3D::set_singleton_for_tests(saved);
}

/* -------------------------------------------------------------- validation */

TEST_CASE("one joint is refused") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();

	ErrorCapture errors;
	CHECK_FALSE(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(1)));
	CHECK(errors.has("A rod needs at least 2 joints, got 1."));

	server->free_rid(body);
}

TEST_CASE("compliance count mismatch") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();
	REQUIRE(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(9)));

	ErrorCapture errors;
	CHECK_FALSE(server->soft_body_set_extra_property(body, K_COMPLIANCE, make_floats(7)));
	CHECK_MESSAGE(errors.has("has 7 entries"), errors.joined());
	CHECK_MESSAGE(errors.has("has 8 segments"), errors.joined());

	server->free_rid(body);
}

TEST_CASE("bend count mismatch") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();
	REQUIRE(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(9)));
	REQUIRE(server->soft_body_set_extra_property(body, K_COMPLIANCE, make_floats(8)));

	ErrorCapture errors;
	CHECK_FALSE(server->soft_body_set_extra_property(body, K_BEND, make_floats(8)));
	CHECK_MESSAGE(errors.has("has 8 entries"), errors.joined());
	CHECK_MESSAGE(errors.has("has 7 bend-twist"), errors.joined());

	server->free_rid(body);
}

TEST_CASE("negative fixed is refused") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();
	REQUIRE(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(9)));

	ErrorCapture errors;
	CHECK_FALSE(server->soft_body_set_extra_property(body, K_FIXED, -1));
	CHECK_MESSAGE(errors.has("[0, 9]"), errors.joined());

	server->free_rid(body);
}

TEST_CASE("over-range fixed is refused") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();
	REQUIRE(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(9)));

	ErrorCapture errors;
	CHECK_FALSE(server->soft_body_set_extra_property(body, K_FIXED, 10));
	CHECK_MESSAGE(errors.has("[0, 9]"), errors.joined());

	server->free_rid(body);
}

TEST_CASE("zero fixed is a free rod") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());

	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9, 0));
	server->soft_body_set_space(body, space.rid());
	// Read after a step, so `_update_mass()` has already rewritten every
	// inverse mass at least once.
	space.step();
	REQUIRE(Probe::in_space(body));

	for (int i = 0; i < 9; i++) {
		CHECK(Probe::vertex_inv_mass(body, i) > 0.0f);
	}

	server->free_rid(body);
}

TEST_CASE("fully pinned rod") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());

	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9, 9));
	server->soft_body_set_space(body, space.rid());
	space.step();
	REQUIRE(Probe::in_space(body));

	for (int i = 0; i < 9; i++) {
		CHECK(Probe::vertex_inv_mass(body, i) == 0.0f);
	}

	server->free_rid(body);
}

TEST_CASE("two joint rod needs no bend") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();

	REQUIRE(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(2)));
	REQUIRE(server->soft_body_set_extra_property(body, K_COMPLIANCE, make_floats(1)));
	// One segment carries zero bend-twist constraints, not -1 of them.
	CHECK(server->soft_body_set_extra_property(body, K_BEND, make_floats(0)));

	ErrorCapture errors;
	CHECK_FALSE(server->soft_body_set_extra_property(body, K_BEND, make_floats(1)));

	server->free_rid(body);
}

TEST_CASE("valid rod is accepted") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();

	CHECK(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(9)));
	CHECK(server->soft_body_set_extra_property(body, K_COMPLIANCE, make_floats(8)));
	CHECK(server->soft_body_set_extra_property(body, K_BEND, make_floats(7)));
	CHECK(server->soft_body_set_extra_property(body, K_FIXED, 1));

	server->free_rid(body);
}

TEST_CASE("zero length segment is refused") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();

	ErrorCapture errors;

	SUBCASE("coincident") {
		PackedVector3Array joints = make_joints(3);
		joints.set(1, joints[0]);
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_JOINTS, joints));
		CHECK_MESSAGE(errors.has("Segment 0"), errors.joined());
	}
	SUBCASE("below the epsilon") {
		PackedVector3Array joints = make_joints(3);
		joints.set(1, joints[0] + Vector3(5e-7, 0, 0));
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_JOINTS, joints));
		CHECK_MESSAGE(errors.has("Segment 0"), errors.joined());
	}
	SUBCASE("above the epsilon") {
		PackedVector3Array joints = make_joints(3);
		joints.set(1, joints[0] + Vector3(2e-6, 0, 0));
		CHECK(server->soft_body_set_extra_property(body, K_JOINTS, joints));
		CHECK(errors.count() == 0);
	}

	server->free_rid(body);
}

TEST_CASE("non finite joints are refused") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();

	ErrorCapture errors;
	real_t value = 0.0;
	SUBCASE("nan") { value = NAN; }
	SUBCASE("inf") { value = INFINITY; }
	SUBCASE("overflowing literal") { value = (real_t)1e400; }

	PackedVector3Array joints = make_joints(4);
	joints.set(2, Vector3(value, 0, 0));
	CHECK_FALSE(server->soft_body_set_extra_property(body, K_JOINTS, joints));
	CHECK_MESSAGE(errors.has("entry 2"), errors.joined());

	server->free_rid(body);
}

TEST_CASE("negative compliance is refused") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();
	REQUIRE(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(9)));

	PackedFloat32Array compliance = make_floats(8);
	compliance.set(3, -1.0f);

	ErrorCapture errors;
	CHECK_FALSE(server->soft_body_set_extra_property(body, K_COMPLIANCE, compliance));
	CHECK_MESSAGE(errors.has("entry 3"), errors.joined());
	CHECK_MESSAGE(errors.has("negative"), errors.joined());

	server->free_rid(body);
}

TEST_CASE("non finite compliance is refused") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();
	REQUIRE(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(9)));
	REQUIRE(server->soft_body_set_extra_property(body, K_COMPLIANCE, make_floats(8)));

	ErrorCapture errors;
	SUBCASE("compliance") {
		PackedFloat32Array values = make_floats(8);
		values.set(2, NAN);
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_COMPLIANCE, values));
	}
	SUBCASE("bend") {
		PackedFloat32Array values = make_floats(7);
		values.set(2, NAN);
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_BEND, values));
	}
	CHECK_MESSAGE(errors.has("entry 2"), errors.joined());
	CHECK_MESSAGE(errors.has("finite"), errors.joined());

	server->free_rid(body);
}

TEST_CASE("fixed before joints is accepted") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();

	ErrorCapture errors;
	// Validation runs over the whole prefix's post-write state, so a scalar may
	// land before the array it is bounded by.
	CHECK(server->soft_body_set_extra_property(body, K_FIXED, 5));
	CHECK(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(9)));
	CHECK(errors.count() == 0);
	CHECK((int)server->soft_body_get_extra_property(body, K_FIXED) == 5);
	CHECK(PackedVector3Array(server->soft_body_get_extra_property(body, K_JOINTS)).size() == 9);

	server->free_rid(body);
}

TEST_CASE("shrinking joints is refused") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9));

	ErrorCapture errors;
	CHECK_FALSE(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(3)));
	CHECK_MESSAGE(errors.has("has 8 entries"), errors.joined());
	CHECK_MESSAGE(errors.has("has 2 segments"), errors.joined());

	server->free_rid(body);
}

TEST_CASE("rejected set leaves state untouched") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9));

	ErrorCapture errors;
	REQUIRE_FALSE(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(3)));

	CHECK(PackedVector3Array(server->soft_body_get_extra_property(body, K_JOINTS)).size() == 9);
	CHECK(PackedFloat32Array(server->soft_body_get_extra_property(body, K_COMPLIANCE)).size() == 8);
	CHECK(PackedFloat32Array(server->soft_body_get_extra_property(body, K_BEND)).size() == 7);
	CHECK((int)server->soft_body_get_extra_property(body, K_FIXED) == 1);
	CHECK(server->soft_body_get_extra_property(body, K_STATE).get_type() == Variant::NIL);

	server->free_rid(body);
}

TEST_CASE("re-setting a key replaces it") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();

	REQUIRE(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(9)));
	REQUIRE(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(4)));

	const PackedVector3Array read = server->soft_body_get_extra_property(body, K_JOINTS);
	CHECK(read.size() == 4);

	server->free_rid(body);
}

TEST_CASE("over-long rod is refused") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();

	ErrorCapture errors;
	CHECK_FALSE(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(1025)));
	CHECK_MESSAGE(errors.has("1024"), errors.joined());

	server->free_rid(body);
}

TEST_CASE("a rod at the cap is accepted") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	RID body = server->soft_body_create();

	ErrorCapture errors;
	CHECK(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(1024)));
	CHECK(errors.count() == 0);

	server->free_rid(body);
}

TEST_CASE("two joint rod orientation is bounded") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());

	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 2));
	server->soft_body_set_space(body, space.rid());
	REQUIRE(Probe::in_space(body));
	// A single segment has no adjacent rod, so it carries no bend-twist
	// constraint at all.
	CHECK(Probe::rod_count(body) == 1);
	CHECK(Probe::rod_bend_twist_count(body) == 0);

	server->soft_body_apply_central_impulse(body, Vector3(0, -20, 0));
	space.step(60);

	const PackedFloat32Array state = server->soft_body_get_extra_property(body, K_STATE);
	REQUIRE(state.size() == 4);
	for (int i = 0; i < state.size(); i++) {
		CHECK(Math::is_finite(state[i]));
	}

	server->free_rid(body);
}

TEST_CASE("fixed without joints does not activate the rod") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());
	RID mesh = make_quad_mesh();

	RID body = server->soft_body_create();
	// `rod/fixed` alone is stored but does not make the capability active: only
	// the required key does.
	REQUIRE(server->soft_body_set_extra_property(body, K_FIXED, 1));
	server->soft_body_set_mesh(body, mesh);
	server->soft_body_set_space(body, space.rid());
	REQUIRE(Probe::in_space(body));
	CHECK(Probe::rod_count(body) == 0);
	CHECK(Probe::face_count(body) == 2);

	REQUIRE(set_valid_rod(server.ptr(), body, 9));
	CHECK(Probe::rod_count(body) == 8);
	CHECK((int)server->soft_body_get_extra_property(body, K_FIXED) == 1);

	server->free_rid(body);
	RenderingServer::get_singleton()->free(mesh);
}

/* ------------------------------------------------------------- arbitration */

TEST_CASE("two replacing capabilities conflict") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());

	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9));
	REQUIRE(server->soft_body_set_extra_property(body, "test_replace/enabled", true));

	ErrorCapture errors;
	server->soft_body_set_space(body, space.rid());
	CHECK_MESSAGE(errors.has("More than one geometry-replacing capability"), errors.joined());
	CHECK_FALSE(Probe::in_space(body));

	server->free_rid(body);
}

TEST_CASE("cloth without a mesh still errors") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());

	RID body = server->soft_body_create();

	ErrorCapture errors;
	server->soft_body_set_space(body, space.rid());
	// Unchanged from vanilla: a mesh-less cloth is inert and says nothing.
	CHECK(errors.count() == 0);
	CHECK_FALSE(Probe::in_space(body));

	server->free_rid(body);
}

TEST_CASE("cloth regression") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());
	RID mesh = make_quad_mesh();

	RID body = server->soft_body_create();
	server->soft_body_set_mesh(body, mesh);
	server->soft_body_set_space(body, space.rid());
	REQUIRE(Probe::in_space(body));

	CHECK(Probe::vertex_count(body) == 4);
	CHECK(Probe::face_count(body) == 2);
	CHECK(Probe::edge_count(body) > 0);
	CHECK(Probe::rod_count(body) == 0);

	const Vector3 before = Probe::vertex_position(body, 0);
	server->soft_body_apply_central_impulse(body, Vector3(0, -20, 0));
	space.step(30);
	CHECK(Probe::vertex_position(body, 0) != before);

	server->free_rid(body);
	RenderingServer::get_singleton()->free(mesh);
}

TEST_CASE("mesh with a rod warns") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());
	RID mesh = make_quad_mesh();

	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9));
	server->soft_body_set_mesh(body, mesh);

	ErrorCapture errors;
	server->soft_body_set_space(body, space.rid());
	CHECK_MESSAGE(errors.has("mesh is ignored"), errors.joined());
	REQUIRE(Probe::in_space(body));
	CHECK(Probe::rod_count(body) == 8);
	CHECK(Probe::face_count(body) == 0);
	CHECK(Probe::vertex_count(body) == 9);

	server->free_rid(body);
	RenderingServer::get_singleton()->free(mesh);
}

/* ------------------------------------------------------------------- traps */

TEST_CASE("optimize makes the rod simulate") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());

	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9));
	server->soft_body_set_space(body, space.rid());
	REQUIRE(Probe::in_space(body));

	const Vector3 before = Probe::vertex_position(body, 8);
	server->soft_body_apply_central_impulse(body, Vector3(0, -20, 0));
	space.step(30);
	CHECK(Probe::vertex_position(body, 8).distance_to(before) > 0.01);

	// `Optimize()` fills the update groups the solver iterates, and an
	// unoptimised body solves no constraint at all -- so the chain would stretch
	// without bound instead of swinging at its rest length.
	const real_t span = Probe::vertex_position(body, 0).distance_to(Probe::vertex_position(body, 8));
	CHECK(span > 7.0);
	CHECK(span < 9.0);

	server->free_rid(body);
}

TEST_CASE("rod without update position drifts") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());

	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9));
	server->soft_body_set_space(body, space.rid());
	REQUIRE(Probe::in_space(body));

	// A rod is anchored to the static world, so the body frame must not chase
	// its own vertices.
	CHECK_FALSE(Probe::update_position(body));

	const Vector3 origin = Probe::center_of_mass(body);
	server->soft_body_apply_central_impulse(body, Vector3(0, -20, 0));
	space.step(60);

	CHECK(Probe::center_of_mass(body).distance_to(origin) < 0.001);
	// The rest length is untouched by the frame decision.
	CHECK(Math::is_equal_approx(Probe::rod_length(body, 0), 1.0f));

	server->free_rid(body);
}

TEST_CASE("update position set too early drifts on rebuild") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());

	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9));
	server->soft_body_set_space(body, space.rid());
	REQUIRE(Probe::in_space(body));
	space.step();

	// The settings object is destroyed once the body exists, so the flag has to
	// be re-applied by `_add_to_space` on every rebuild, not at settings build.
	REQUIRE(server->soft_body_set_extra_property(body, K_COMPLIANCE, make_floats(8, 0.5f)));
	REQUIRE(Probe::in_space(body));
	CHECK_FALSE(Probe::update_position(body));

	const Vector3 origin = Probe::center_of_mass(body);
	server->soft_body_apply_central_impulse(body, Vector3(0, -20, 0));
	space.step(60);
	CHECK(Probe::center_of_mass(body).distance_to(origin) < 0.001);

	server->free_rid(body);
}

TEST_CASE("mass update preserves pins") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());

	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9, 2));
	server->soft_body_set_space(body, space.rid());
	REQUIRE(Probe::in_space(body));

	// `_update_mass()` rewrites every inverse mass, so the pins have to be
	// re-applied behind it.
	server->soft_body_set_total_mass(body, 5.0);
	space.step();

	CHECK(Probe::vertex_inv_mass(body, 0) == 0.0f);
	CHECK(Probe::vertex_inv_mass(body, 1) == 0.0f);
	CHECK(Probe::vertex_inv_mass(body, 2) > 0.0f);

	server->free_rid(body);
}

TEST_CASE("calculate rod properties keeps chain order") {
	CHECK(Probe::calculate_rod_properties_keeps_chain_order(9));
}

TEST_CASE("optimize is identity for a nine vertex chain") {
	CHECK(Probe::optimize_is_identity_for_chain(9));
}

TEST_CASE("inverse mass matches update mass") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());

	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9, 0));
	server->soft_body_set_total_mass(body, 1.0);
	server->soft_body_set_space(body, space.rid());
	REQUIRE(Probe::in_space(body));
	space.step();

	// The cloth path's formula, not 1.0: vertex count over total mass.
	CHECK(Math::is_equal_approx(Probe::vertex_inv_mass(body, 4), 9.0f));

	server->free_rid(body);
}

TEST_CASE("rebuild rederives rod properties") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());

	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9));
	server->soft_body_set_space(body, space.rid());
	REQUIRE(Probe::in_space(body));
	CHECK(Math::is_equal_approx(Probe::rod_length(body, 0), 1.0f));

	// A reused settings object would short-circuit `CalculateRodProperties`,
	// leaving the previous build's lengths behind.
	REQUIRE(server->soft_body_set_extra_property(body, K_JOINTS, make_joints(9, 2.0)));
	REQUIRE(Probe::in_space(body));
	CHECK(Math::is_equal_approx(Probe::rod_length(body, 0), 2.0f));
	CHECK(Probe::all_bishop_frames_set(body));

	server->free_rid(body);
}

TEST_CASE("state order survives optimize at scale") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());

	const int joint_count = 200;
	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, joint_count));
	server->soft_body_set_space(body, space.rid());
	REQUIRE(Probe::in_space(body));
	REQUIRE(Probe::rod_count(body) == joint_count - 1);

	// Every segment of the input chain points along +X, and the Bishop frame's
	// Z axis is the segment tangent -- so `rod/state[i]` must map +Z to +X for
	// every `i`, which no permuted ordering would satisfy element-by-element.
	for (int i = 0; i < joint_count - 1; i++) {
		CHECK(Probe::rod_vertex(body, i, 0) == i);
		CHECK(Probe::rod_vertex(body, i, 1) == i + 1);

		const Quaternion rotation = read_rod_rotation(server.ptr(), body, i);
		const Vector3 tangent = rotation.xform(Vector3(0, 0, 1));
		CHECK(tangent.distance_to(Vector3(1, 0, 0)) < 0.01);
	}

	server->free_rid(body);
}

/* --------------------------------------------------------------- derivation */

TEST_CASE("derive populates rod properties") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());

	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9));
	server->soft_body_set_space(body, space.rid());
	REQUIRE(Probe::in_space(body));

	CHECK(Probe::min_rod_length(body) > 0.0f);
	CHECK(Probe::all_bishop_frames_set(body));

	server->free_rid(body);
}

TEST_CASE("rod states are sized by segment") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());

	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9));
	server->soft_body_set_space(body, space.rid());
	REQUIRE(Probe::in_space(body));

	// Segments, not particles: 9 joints give 8 rods.
	CHECK(Probe::vertex_count(body) == 9);
	CHECK(Probe::rod_count(body) == 8);
	CHECK(Probe::rod_state_is_readable(body, 7));
	CHECK(PackedFloat32Array(server->soft_body_get_extra_property(body, K_STATE)).size() == 8 * 4);

	server->free_rid(body);
}

TEST_CASE("one pin frees orientation two pins fix it") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());

	RID free_end = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), free_end, 9, 1));
	server->soft_body_set_space(free_end, space.rid());
	REQUIRE(Probe::in_space(free_end));

	RID clamped = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), clamped, 9, 2));
	server->soft_body_set_space(clamped, space.rid());
	REQUIRE(Probe::in_space(clamped));

	const Quaternion clamped_before = read_rod_rotation(server.ptr(), clamped, 0);

	server->soft_body_apply_central_impulse(free_end, Vector3(0, -40, 0));
	server->soft_body_apply_central_impulse(clamped, Vector3(0, -40, 0));
	space.step(60);

	// `rod/fixed` pins the first N joints, and rod 0 spans joints 0 and 1 -- so one
	// pin leaves the rod's far end free to move while two nail both of its ends.
	// The assertions are moved-at-all versus not-moved-at-all rather than an angle:
	// the swing right next to an anchor is tiny (measured 0.014 units) and it damps
	// away over a few seconds, while the clamped body's numbers stay bit-identical
	// to their initial ones.
	CHECK(Probe::vertex_position(free_end, 1) != Vector3(1, 0, 0));
	CHECK(Probe::vertex_position(clamped, 1) == Vector3(1, 0, 0));

	// Measured: rod 0's stored rotation is bit-identical in BOTH bodies -- with the
	// far end moving by 0.014 the solver's angular velocity stays at zero to float
	// precision, so `rod/state` cannot carry the free-versus-fixed claim and only
	// the clamped side is asserted through it. `soft_body_apply_point_impulse` is
	// not an alternative excitation: it indexes `mesh_to_physics`, which a rod (no
	// mesh) leaves empty.
	CHECK(read_rod_rotation(server.ptr(), clamped, 0) == clamped_before);

	server->free_rid(clamped);
	server->free_rid(free_end);
}

/* --------------------------------------------------------------- lifecycle */

TEST_CASE("set mesh keeps capability state") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());
	RID mesh = make_quad_mesh();

	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9));

	ErrorCapture errors;
	server->soft_body_set_mesh(body, mesh);
	server->soft_body_set_space(body, space.rid());
	REQUIRE(Probe::in_space(body));

	CHECK(PackedVector3Array(server->soft_body_get_extra_property(body, K_JOINTS)).size() == 9);
	CHECK(Probe::rod_count(body) == 8);

	server->free_rid(body);
	RenderingServer::get_singleton()->free(mesh);
}

TEST_CASE("space change keeps capability state") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());

	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9));
	server->soft_body_set_space(body, space.rid());
	REQUIRE(Probe::in_space(body));

	server->soft_body_set_space(body, RID());
	CHECK_FALSE(Probe::in_space(body));

	server->soft_body_set_space(body, space.rid());
	REQUIRE(Probe::in_space(body));
	CHECK(Probe::rod_count(body) == 8);
	CHECK(PackedVector3Array(server->soft_body_get_extra_property(body, K_JOINTS)).size() == 9);

	server->free_rid(body);
}

TEST_CASE("new body starts with no capability state") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());

	RID first = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), first, 9));
	server->soft_body_set_space(first, space.rid());
	REQUIRE(Probe::in_space(first));
	server->free_rid(first);

	RID second = server->soft_body_create();
	CHECK(server->soft_body_get_extra_property(second, K_JOINTS).get_type() == Variant::NIL);
	CHECK(server->soft_body_get_extra_property(second, K_FIXED).get_type() == Variant::NIL);
	CHECK(server->soft_body_get_extra_property_list(second).size() >= 5);

	server->free_rid(second);
}

TEST_CASE("live parameter change rebuilds") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());

	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9));
	server->soft_body_set_space(body, space.rid());
	REQUIRE(Probe::in_space(body));
	CHECK(Probe::rod_compliance(body, 0) == 0.0f);

	REQUIRE(server->soft_body_set_extra_property(body, K_COMPLIANCE, make_floats(8, 0.25f)));
	space.step();
	REQUIRE(Probe::in_space(body));
	CHECK(Math::is_equal_approx(Probe::rod_compliance(body, 0), 0.25f));

	server->free_rid(body);
}

TEST_CASE("cloth vertex api rejects a rod") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());

	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9, 2));
	server->soft_body_set_space(body, space.rid());
	REQUIRE(Probe::in_space(body));

	ErrorCapture errors;
	// A rod has no mesh, so the mesh-indexed cloth API has nothing to address.
	server->soft_body_move_point(body, 0, Vector3(1, 1, 1));
	CHECK(server->soft_body_get_point_global_position(body, 0) == Vector3());
	server->soft_body_pin_point(body, 0, true);
	CHECK_FALSE(server->soft_body_is_point_pinned(body, 0));
	server->soft_body_apply_point_impulse(body, 0, Vector3(0, 1, 0));
	CHECK(errors.count() > 0);

	space.step();
	CHECK(Probe::vertex_inv_mass(body, 0) == 0.0f);
	CHECK(Probe::vertex_inv_mass(body, 1) == 0.0f);
	CHECK(Probe::vertex_inv_mass(body, 2) > 0.0f);

	server->free_rid(body);
}

TEST_CASE("cloth setters do not disturb a rod") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());
	SpaceScope space(server.ptr());

	RID body = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), body, 9));
	server->soft_body_set_space(body, space.rid());
	REQUIRE(Probe::in_space(body));

	const PackedVector3Array joints_before = server->soft_body_get_extra_property(body, K_JOINTS);
	const PackedFloat32Array compliance_before = server->soft_body_get_extra_property(body, K_COMPLIANCE);
	const PackedFloat32Array bend_before = server->soft_body_get_extra_property(body, K_BEND);
	const int fixed_before = server->soft_body_get_extra_property(body, K_FIXED);
	const PackedFloat32Array state_before = server->soft_body_get_extra_property(body, K_STATE);

	// Pre-existing public API this FR changes in no way: every one is accepted
	// and reads back.
	server->soft_body_set_simulation_precision(body, 7);
	server->soft_body_set_total_mass(body, 3.0);
	server->soft_body_set_linear_stiffness(body, 0.25);
	server->soft_body_set_shrinking_factor(body, 0.5);
	server->soft_body_set_pressure_coefficient(body, 2.0);
	server->soft_body_set_damping_coefficient(body, 0.75);

	CHECK(server->soft_body_get_simulation_precision(body) == 7);
	CHECK(Math::is_equal_approx(server->soft_body_get_total_mass(body), (real_t)3.0));
	CHECK(Math::is_equal_approx(server->soft_body_get_linear_stiffness(body), (real_t)0.25));
	CHECK(Math::is_equal_approx(server->soft_body_get_shrinking_factor(body), (real_t)0.5));
	CHECK(Math::is_equal_approx(server->soft_body_get_pressure_coefficient(body), (real_t)2.0));
	CHECK(Math::is_equal_approx(server->soft_body_get_damping_coefficient(body), (real_t)0.75));

	CHECK(PackedVector3Array(server->soft_body_get_extra_property(body, K_JOINTS)) == joints_before);
	CHECK(PackedFloat32Array(server->soft_body_get_extra_property(body, K_COMPLIANCE)) == compliance_before);
	CHECK(PackedFloat32Array(server->soft_body_get_extra_property(body, K_BEND)) == bend_before);
	CHECK((int)server->soft_body_get_extra_property(body, K_FIXED) == fixed_before);
	CHECK(PackedFloat32Array(server->soft_body_get_extra_property(body, K_STATE)) == state_before);

	server->free_rid(body);
}

TEST_CASE("free before build leaks nothing") {
	ServerScope server("Jolt Physics");
	REQUIRE(server.is_valid());

	RID first = server->soft_body_create();
	REQUIRE(set_valid_rod(server.ptr(), first, 9));
	// Freed without ever entering a space, so nothing was ever built.
	server->free_rid(first);

	RID second = server->soft_body_create();
	CHECK(server->soft_body_get_extra_property(second, K_JOINTS).get_type() == Variant::NIL);
	CHECK(server->soft_body_get_extra_property(second, K_COMPLIANCE).get_type() == Variant::NIL);

	server->free_rid(second);
}

/* ------------------------------------------------------------------ registry */

TEST_CASE("every capability file is listed") {
	const int file_count = Probe::capability_source_file_count();
	REQUIRE(file_count > 0);

	HashSet<String> listed;
	for (int i = 0; i < Probe::capability_count(); i++) {
		listed.insert(Probe::capability_prefix(i));
	}

	// `capabilities/cap_<x>.cpp` owns prefix `<x>/`. A file that self-registers
	// into no list would be silently inert.
	for (int i = 0; i < file_count; i++) {
		const String file = Probe::capability_source_file(i);
		REQUIRE(file.begins_with("cap_"));
		REQUIRE(file.ends_with(".cpp"));
		const String prefix = file.substr(4, file.length() - 8) + "/";
		CHECK_MESSAGE(listed.has(prefix), file);
	}
	CHECK(Probe::capability_count() == file_count);
}

TEST_CASE("every capability names a writable required key") {
	REQUIRE(Probe::capability_count() > 0);

	for (int cap = 0; cap < Probe::capability_count(); cap++) {
		const String required = Probe::capability_required_key(cap);
		CHECK_MESSAGE(required.begins_with(Probe::capability_prefix(cap)), required);

		bool found = false;
		for (int prop = 0; prop < Probe::capability_property_count(cap); prop++) {
			if (Probe::capability_property_name(cap, prop) == required) {
				found = true;
				// Activity is decided by a key an author can write; a read-only
				// key could never turn the capability on.
				CHECK_FALSE(Probe::capability_property_live_read(cap, prop));
			}
		}
		CHECK_MESSAGE(found, required);
	}
}

} // TEST_SUITE

} // namespace TestJoltSoftBodyCap
