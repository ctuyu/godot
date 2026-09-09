/**************************************************************************/
/*  soft_body_cap_probe.cpp                                               */
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

#ifdef TESTS_ENABLED

#include "soft_body_cap_probe.h"

#include "../capabilities/soft_body_capabilities.h"
#include "../jolt_physics_server_3d.h"
#include "../objects/jolt_soft_body_3d.h"

#include "core/math/math_funcs.h"

#include <cfloat>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>

namespace {

const JPH::SoftBodyMotionProperties *resolve(RID p_body) {
	JoltPhysicsServer3D *server = JoltPhysicsServer3D::get_singleton();
	if (server == nullptr) {
		return nullptr;
	}
	JoltSoftBody3D *body = server->get_soft_body_for_tests(p_body);
	if (body == nullptr || !body->in_space()) {
		return nullptr;
	}
	return static_cast<const JPH::SoftBodyMotionProperties *>(body->get_jolt_body()->GetMotionPropertiesUnchecked());
}

// A chain of `p_joints` particles: N-1 stretch-shear rods, N-2 bend-twist. Laid
// out along +X with unit spacing so every derived length is exactly 1.
void build_chain(JPH::SoftBodySharedSettings &r_settings, int p_joints) {
	for (int i = 0; i < p_joints; i++) {
		JPH::SoftBodySharedSettings::Vertex vertex;
		vertex.mPosition = JPH::Float3((float)i, 0.0f, 0.0f);
		vertex.mInvMass = 1.0f;
		r_settings.mVertices.push_back(vertex);
	}
	for (int i = 0; i + 1 < p_joints; i++) {
		r_settings.mRodStretchShearConstraints.push_back(JPH::SoftBodySharedSettings::RodStretchShear((JPH::uint32)i, (JPH::uint32)(i + 1), 0.0f));
	}
	for (int i = 0; i + 2 < p_joints; i++) {
		r_settings.mRodBendTwistConstraints.push_back(JPH::SoftBodySharedSettings::RodBendTwist((JPH::uint32)i, (JPH::uint32)(i + 1), 0.0f));
	}
}

} // namespace

namespace JoltSoftBodyCapProbe {

bool in_space(RID p_body) {
	return resolve(p_body) != nullptr;
}

int vertex_count(RID p_body) {
	const JPH::SoftBodyMotionProperties *motion = resolve(p_body);
	return motion == nullptr ? -1 : (int)motion->GetVertices().size();
}

float vertex_inv_mass(RID p_body, int p_index) {
	const JPH::SoftBodyMotionProperties *motion = resolve(p_body);
	if (motion == nullptr || p_index < 0 || p_index >= (int)motion->GetVertices().size()) {
		return -1.0f;
	}
	return motion->GetVertex((JPH::uint)p_index).mInvMass;
}

Vector3 vertex_position(RID p_body, int p_index) {
	const JPH::SoftBodyMotionProperties *motion = resolve(p_body);
	if (motion == nullptr || p_index < 0 || p_index >= (int)motion->GetVertices().size()) {
		return Vector3();
	}
	const JPH::Vec3 position = motion->GetVertex((JPH::uint)p_index).mPosition;
	return Vector3(position.GetX(), position.GetY(), position.GetZ());
}

Vector3 center_of_mass(RID p_body) {
	JoltPhysicsServer3D *server = JoltPhysicsServer3D::get_singleton();
	if (server == nullptr) {
		return Vector3();
	}
	JoltSoftBody3D *body = server->get_soft_body_for_tests(p_body);
	if (body == nullptr || !body->in_space()) {
		return Vector3();
	}
	const JPH::RVec3 position = body->get_jolt_body()->GetCenterOfMassPosition();
	return Vector3((real_t)position.GetX(), (real_t)position.GetY(), (real_t)position.GetZ());
}

int face_count(RID p_body) {
	const JPH::SoftBodyMotionProperties *motion = resolve(p_body);
	return motion == nullptr ? -1 : (int)motion->GetSettings()->mFaces.size();
}

int edge_count(RID p_body) {
	const JPH::SoftBodyMotionProperties *motion = resolve(p_body);
	return motion == nullptr ? -1 : (int)motion->GetSettings()->mEdgeConstraints.size();
}

int rod_count(RID p_body) {
	const JPH::SoftBodyMotionProperties *motion = resolve(p_body);
	return motion == nullptr ? -1 : (int)motion->GetSettings()->mRodStretchShearConstraints.size();
}

int rod_bend_twist_count(RID p_body) {
	const JPH::SoftBodyMotionProperties *motion = resolve(p_body);
	return motion == nullptr ? -1 : (int)motion->GetSettings()->mRodBendTwistConstraints.size();
}

bool rod_state_is_readable(RID p_body, int p_index) {
	const JPH::SoftBodyMotionProperties *motion = resolve(p_body);
	if (motion == nullptr || p_index < 0 || p_index >= (int)motion->GetSettings()->mRodStretchShearConstraints.size()) {
		return false;
	}
	// Reading the state at all is the assertion: `mRodStates` is sized by
	// `SoftBodyMotionProperties::Initialize` from the constraint count, so an
	// undersized array would be an out-of-bounds read here, not a wrong value.
	return motion->GetRodRotation((JPH::uint)p_index).IsNormalized();
}

float rod_length(RID p_body, int p_index) {
	const JPH::SoftBodyMotionProperties *motion = resolve(p_body);
	if (motion == nullptr || p_index < 0 || p_index >= (int)motion->GetSettings()->mRodStretchShearConstraints.size()) {
		return -1.0f;
	}
	return motion->GetSettings()->mRodStretchShearConstraints[p_index].mLength;
}

float min_rod_length(RID p_body) {
	const JPH::SoftBodyMotionProperties *motion = resolve(p_body);
	if (motion == nullptr) {
		return -1.0f;
	}
	float smallest = FLT_MAX;
	for (const JPH::SoftBodySharedSettings::RodStretchShear &rod : motion->GetSettings()->mRodStretchShearConstraints) {
		smallest = MIN(smallest, rod.mLength);
	}
	return smallest == FLT_MAX ? -1.0f : smallest;
}

bool all_bishop_frames_set(RID p_body) {
	const JPH::SoftBodyMotionProperties *motion = resolve(p_body);
	if (motion == nullptr) {
		return false;
	}
	const JPH::Array<JPH::SoftBodySharedSettings::RodStretchShear> &rods = motion->GetSettings()->mRodStretchShearConstraints;
	if (rods.empty()) {
		return false;
	}
	for (const JPH::SoftBodySharedSettings::RodStretchShear &rod : rods) {
		// `CalculateRodProperties` leaves a zero quaternion behind when it has not
		// run, and produces a NaN one when a segment has zero length.
		if (rod.mBishop == JPH::Quat::sZero() || !rod.mBishop.IsNormalized()) {
			return false;
		}
	}
	return true;
}

bool rod_angular_velocities_are_zero(RID p_body) {
	const JPH::SoftBodyMotionProperties *motion = resolve(p_body);
	if (motion == nullptr) {
		return false;
	}
	const int count = (int)motion->GetSettings()->mRodStretchShearConstraints.size();
	for (int i = 0; i < count; i++) {
		if (!motion->GetRodAngularVelocity((JPH::uint)i).IsNearZero()) {
			return false;
		}
	}
	return true;
}

int rod_vertex(RID p_body, int p_rod, int p_which) {
	const JPH::SoftBodyMotionProperties *motion = resolve(p_body);
	if (motion == nullptr || p_which < 0 || p_which > 1) {
		return -1;
	}
	const JPH::Array<JPH::SoftBodySharedSettings::RodStretchShear> &rods = motion->GetSettings()->mRodStretchShearConstraints;
	if (p_rod < 0 || p_rod >= (int)rods.size()) {
		return -1;
	}
	return (int)rods[p_rod].mVertex[p_which];
}

int bend_twist_rod(RID p_body, int p_bend, int p_which) {
	const JPH::SoftBodyMotionProperties *motion = resolve(p_body);
	if (motion == nullptr || p_which < 0 || p_which > 1) {
		return -1;
	}
	const JPH::Array<JPH::SoftBodySharedSettings::RodBendTwist> &bends = motion->GetSettings()->mRodBendTwistConstraints;
	if (p_bend < 0 || p_bend >= (int)bends.size()) {
		return -1;
	}
	return (int)bends[p_bend].mRod[p_which];
}

float rod_compliance(RID p_body, int p_index) {
	const JPH::SoftBodyMotionProperties *motion = resolve(p_body);
	if (motion == nullptr || p_index < 0 || p_index >= (int)motion->GetSettings()->mRodStretchShearConstraints.size()) {
		return -1.0f;
	}
	return motion->GetSettings()->mRodStretchShearConstraints[p_index].mCompliance;
}

float bend_twist_compliance(RID p_body, int p_index) {
	const JPH::SoftBodyMotionProperties *motion = resolve(p_body);
	if (motion == nullptr || p_index < 0 || p_index >= (int)motion->GetSettings()->mRodBendTwistConstraints.size()) {
		return -1.0f;
	}
	return motion->GetSettings()->mRodBendTwistConstraints[p_index].mCompliance;
}

bool update_position(RID p_body) {
	const JPH::SoftBodyMotionProperties *motion = resolve(p_body);
	return motion == nullptr ? false : motion->GetUpdatePosition();
}

bool is_inner_jolt_server(const void *p_server) {
	return (const void *)JoltPhysicsServer3D::get_singleton() == p_server;
}

int capability_count() {
	return (int)all_capabilities().size();
}

String capability_prefix(int p_cap) {
	ERR_FAIL_INDEX_V(p_cap, capability_count(), String());
	return String(all_capabilities()[p_cap]->prefix);
}

String capability_required_key(int p_cap) {
	ERR_FAIL_INDEX_V(p_cap, capability_count(), String());
	return String(all_capabilities()[p_cap]->required_key);
}

int capability_property_count(int p_cap) {
	ERR_FAIL_INDEX_V(p_cap, capability_count(), -1);
	return (int)all_capabilities()[p_cap]->props.size();
}

String capability_property_name(int p_cap, int p_prop) {
	ERR_FAIL_INDEX_V(p_cap, capability_count(), String());
	ERR_FAIL_INDEX_V(p_prop, capability_property_count(p_cap), String());
	return String(all_capabilities()[p_cap]->props[p_prop].name);
}

int capability_property_type(int p_cap, int p_prop) {
	ERR_FAIL_INDEX_V(p_cap, capability_count(), -1);
	ERR_FAIL_INDEX_V(p_prop, capability_property_count(p_cap), -1);
	return (int)all_capabilities()[p_cap]->props[p_prop].type;
}

bool capability_property_live_read(int p_cap, int p_prop) {
	ERR_FAIL_INDEX_V(p_cap, capability_count(), false);
	ERR_FAIL_INDEX_V(p_prop, capability_property_count(p_cap), false);
	return all_capabilities()[p_cap]->props[p_prop].live_read;
}

int capability_source_file_count() {
	return (int)(sizeof(CAP_SOURCE_FILES) / sizeof(CAP_SOURCE_FILES[0]));
}

String capability_source_file(int p_index) {
	ERR_FAIL_INDEX_V(p_index, capability_source_file_count(), String());
	return String(CAP_SOURCE_FILES[p_index]);
}

bool calculate_rod_properties_keeps_chain_order(int p_joints) {
	if (p_joints < 2) {
		return false;
	}
	JPH::SoftBodySharedSettings settings;
	build_chain(settings, p_joints);
	settings.CalculateRodProperties();

	const JPH::Array<JPH::SoftBodySharedSettings::RodStretchShear> &rods = settings.mRodStretchShearConstraints;
	for (int i = 0; i < (int)rods.size(); i++) {
		if (rods[i].mVertex[0] != (JPH::uint32)i || rods[i].mVertex[1] != (JPH::uint32)(i + 1)) {
			return false;
		}
		// Every segment of the chain is one unit long, laid out along +X.
		if (!Math::is_equal_approx((real_t)rods[i].mLength, (real_t)1.0)) {
			return false;
		}
		if (rods[i].mBishop == JPH::Quat::sZero() || !rods[i].mBishop.IsNormalized()) {
			return false;
		}
		// The Bishop frame's Z axis is the segment tangent
		// (`SoftBodySharedSettings.cpp`: the tangent is the third column).
		const JPH::Vec3 tangent = rods[i].mBishop * JPH::Vec3(0, 0, 1);
		if (!Math::is_equal_approx((real_t)tangent.GetX(), (real_t)1.0, (real_t)1e-4)) {
			return false;
		}
	}
	const JPH::Array<JPH::SoftBodySharedSettings::RodBendTwist> &bends = settings.mRodBendTwistConstraints;
	for (int i = 0; i < (int)bends.size(); i++) {
		if (bends[i].mRod[0] != (JPH::uint32)i || bends[i].mRod[1] != (JPH::uint32)(i + 1)) {
			return false;
		}
		// A straight chain twists by nothing between adjacent rods.
		if (!bends[i].mOmega0.IsNormalized()) {
			return false;
		}
	}
	return true;
}

bool optimize_is_identity_for_chain(int p_joints) {
	if (p_joints < 2) {
		return false;
	}
	JPH::SoftBodySharedSettings settings;
	build_chain(settings, p_joints);
	settings.CalculateRodProperties();

	JPH::SoftBodySharedSettings::OptimizationResults results;
	settings.Optimize(results);

	// The stretch-shear remap is the identity, which is the half that matters:
	// `rod/state[i]` is `mRodStates[i]`, so a permutation here would silently
	// renumber every rod an author addressed.
	if (results.mRodStretchShearConstraintRemap.size() != (size_t)(p_joints - 1)) {
		return false;
	}
	for (JPH::uint i = 0; i < results.mRodStretchShearConstraintRemap.size(); i++) {
		if (results.mRodStretchShearConstraintRemap[i] != i) {
			return false;
		}
	}

	// Measured: the bend-twist remap is NOT the identity even for a straight chain
	// (a 9-joint chain yields `0 6 2 3 4 5 1`). Nothing exposes bend-twist order,
	// so what has to survive is the invariant: a permutation, and every bend still
	// names two adjacent rods.
	const size_t bend_count = (size_t)MAX(p_joints - 2, 0);
	if (results.mRodBendTwistConstraintRemap.size() != bend_count) {
		return false;
	}
	JPH::Array<bool> seen;
	seen.resize(bend_count, false);
	for (JPH::uint i = 0; i < results.mRodBendTwistConstraintRemap.size(); i++) {
		const JPH::uint mapped = results.mRodBendTwistConstraintRemap[i];
		if (mapped >= (JPH::uint)bend_count || seen[mapped]) {
			return false;
		}
		seen[mapped] = true;
	}
	for (const JPH::SoftBodySharedSettings::RodBendTwist &bend : settings.mRodBendTwistConstraints) {
		if (bend.mRod[1] != bend.mRod[0] + 1) {
			return false;
		}
	}
	return true;
}

} // namespace JoltSoftBodyCapProbe

#endif // TESTS_ENABLED
