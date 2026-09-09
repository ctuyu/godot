/**************************************************************************/
/*  cap_rod.cpp                                                           */
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

#include "soft_body_capabilities.h"

#include "soft_body_cap_state.h"

#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>

namespace {

const char *KEY_JOINTS = "rod/joints";
const char *KEY_COMPLIANCE = "rod/compliance";
const char *KEY_BEND = "rod/bend";
const char *KEY_FIXED = "rod/fixed";
const char *KEY_STATE = "rod/state";

// The remap `Optimize()` publishes is only measured up to two orders of magnitude
// below this, and "segment i is bone i" rests on that remap being the identity.
// The cap turns the measured range into a precondition the API enforces.
constexpr int MAX_JOINTS = 1024;

// Squared, so the check is one multiply. Frozen rather than "a small epsilon":
// a zero-length segment divides by its own length inside Jolt, under an assert
// that a release build does not compile, so the threshold is the whole guard.
constexpr double MIN_SEGMENT_LENGTH_SQUARED = 1e-12;

bool validate_finite_array(const PackedFloat32Array &p_values, const char *p_key, String *r_err) {
	for (int i = 0; i < p_values.size(); ++i) {
		const float value = p_values[i];

		if (!Math::is_finite(value)) {
			*r_err = vformat("'%s' entry %d is not a finite number (%f).", p_key, i, value);
			return false;
		}

		// Compliance is the solver's denominator, so a negative entry is not
		// "extra stiff", it is garbage.
		if (value < 0.0f) {
			*r_err = vformat("'%s' entry %d must not be negative, got %f.", p_key, i, value);
			return false;
		}
	}

	return true;
}

bool rod_validate(const JoltSoftBodyCapState &p_state, String *r_err) {
	const bool has_joints = p_state.has(KEY_JOINTS);
	const PackedVector3Array joints = p_state.v3(KEY_JOINTS);
	const int joint_count = joints.size();

	if (has_joints) {
		if (joint_count < 2) {
			*r_err = vformat("A rod needs at least 2 joints, got %d.", joint_count);
			return false;
		}

		if (joint_count > MAX_JOINTS) {
			*r_err = vformat("A rod may have at most %d joints, got %d. This is a deliberate scope limit: the constraint reordering a rod's state order depends on is only measured below it.", MAX_JOINTS, joint_count);
			return false;
		}

		for (int i = 0; i < joint_count; ++i) {
			const Vector3 joint = joints[i];

			if (!Math::is_finite(joint.x) || !Math::is_finite(joint.y) || !Math::is_finite(joint.z)) {
				*r_err = vformat("'%s' entry %d is not finite (%v).", KEY_JOINTS, i, joint);
				return false;
			}
		}

		for (int i = 0; i + 1 < joint_count; ++i) {
			const double length_squared = (double)joints[i + 1].distance_squared_to(joints[i]);

			if (length_squared < MIN_SEGMENT_LENGTH_SQUARED) {
				*r_err = vformat("Segment %d of '%s' has a length of %f, which is below the minimum of %f.", i, KEY_JOINTS, Math::sqrt(length_squared), Math::sqrt(MIN_SEGMENT_LENGTH_SQUARED));
				return false;
			}
		}
	}

	// `rod/joints` may legitimately be absent: activity is its presence, not
	// validity's business, so a cross-key rule is checked only when both of its
	// operands are there.
	const int rods = has_joints ? joint_count - 1 : 0;

	if (p_state.has(KEY_COMPLIANCE)) {
		const PackedFloat32Array compliance = p_state.f32(KEY_COMPLIANCE);

		if (!validate_finite_array(compliance, KEY_COMPLIANCE, r_err)) {
			return false;
		}

		if (has_joints && compliance.size() != rods) {
			*r_err = vformat("'%s' has %d entries but a rod with %d joints has %d segments.", KEY_COMPLIANCE, compliance.size(), joint_count, rods);
			return false;
		}
	}

	if (p_state.has(KEY_BEND)) {
		const PackedFloat32Array bend = p_state.f32(KEY_BEND);

		if (!validate_finite_array(bend, KEY_BEND, r_err)) {
			return false;
		}

		// Bend-twist constraints join two segments, so there is one fewer of them
		// than there are segments -- and none at all for a single-segment rod.
		const int expected_bend = rods > 0 ? rods - 1 : 0;

		if (has_joints && bend.size() != expected_bend) {
			*r_err = vformat("'%s' has %d entries but a rod with %d segments has %d bend-twist constraints.", KEY_BEND, bend.size(), rods, expected_bend);
			return false;
		}
	}

	if (p_state.has(KEY_FIXED)) {
		const int fixed = p_state.i(KEY_FIXED);

		if (has_joints) {
			if (fixed < 0 || fixed > joint_count) {
				*r_err = vformat("'%s' must be in the range [0, %d], got %d.", KEY_FIXED, joint_count, fixed);
				return false;
			}
		} else if (fixed < 0) {
			*r_err = vformat("'%s' must not be negative, got %d.", KEY_FIXED, fixed);
			return false;
		}
	}

	return true;
}

// The single source of truth for which vertices a rod pins, read both at build
// time and by the re-pin that follows a mass recomputation.
void rod_pinned_vertices(const JoltSoftBodyCapState &p_state, LocalVector<int> &r_indices) {
	const int fixed = p_state.i(KEY_FIXED);

	for (int i = 0; i < fixed; ++i) {
		r_indices.push_back(i);
	}
}

void rod_contribute(const JoltSoftBodyCapState &p_state, JPH::SoftBodySharedSettings &r_settings, const LocalVector<int> &p_mesh_to_physics, float p_mass) {
	const PackedVector3Array joints = p_state.v3(KEY_JOINTS);
	const int joint_count = joints.size();
	ERR_FAIL_COND(joint_count < 2);

	const int rods = joint_count - 1;
	const PackedFloat32Array compliance = p_state.f32(KEY_COMPLIANCE);
	const PackedFloat32Array bend = p_state.f32(KEY_BEND);

	JPH::Array<JPH::SoftBodySharedSettings::Vertex> &vertices = r_settings.mVertices;
	const int vertex_base = (int)vertices.size();

	for (int i = 0; i < joint_count; ++i) {
		const Vector3 joint = joints[i];
		vertices.emplace_back(JPH::Float3((float)joint.x, (float)joint.y, (float)joint.z));
	}

	// The same value `_update_mass()` will write one step later. Anything else and
	// the settings disagree with the body, and the rod's derived inverse masses
	// are computed from the vertices as they stand here.
	const float inverse_vertex_mass = (float)vertices.size() / p_mass;
	for (int i = 0; i < joint_count; ++i) {
		vertices[vertex_base + i].mInvMass = inverse_vertex_mass;
	}

	JPH::Array<JPH::SoftBodySharedSettings::RodStretchShear> &stretch_shear = r_settings.mRodStretchShearConstraints;
	const int rod_base = (int)stretch_shear.size();

	// Stretch-shear indexes particles.
	for (int i = 0; i < rods; ++i) {
		const float value = i < compliance.size() ? compliance[i] : 0.0f;
		stretch_shear.emplace_back((JPH::uint32)(vertex_base + i), (JPH::uint32)(vertex_base + i + 1), value);
	}

	// Bend-twist indexes segments -- the only constraint type that does not index
	// particles. An off-by-one here yields a plausible rod with the wrong
	// stiffness distribution and no diagnostic.
	JPH::Array<JPH::SoftBodySharedSettings::RodBendTwist> &bend_twist = r_settings.mRodBendTwistConstraints;
	for (int i = 0; i + 1 < rods; ++i) {
		const float value = i < bend.size() ? bend[i] : 0.0f;
		bend_twist.emplace_back((JPH::uint32)(rod_base + i), (JPH::uint32)(rod_base + i + 1), value);
	}

	LocalVector<int> pinned;
	rod_pinned_vertices(p_state, pinned);
	for (int index : pinned) {
		if (index >= 0 && index < joint_count) {
			vertices[vertex_base + index].mInvMass = 0.0f;
		}
	}
}

void rod_derive(JPH::SoftBodySharedSettings &r_settings) {
	r_settings.CalculateRodProperties();
}

void rod_apply_body(const JoltSoftBodyCapState &p_state, JPH::SoftBodyCreationSettings &r_settings) {
	// A rod's centre of mass is not the thing being simulated, and letting Jolt
	// move the body's origin to follow it makes every read of a rod's state drift
	// against the coordinates it was built in.
	r_settings.mUpdatePosition = false;
}

Variant rod_get(const JoltSoftBodyCapState &p_state, const JPH::Body *p_body, const StringName &p_name) {
	if (p_name != StringName(KEY_STATE)) {
		return p_state.get(p_name);
	}

	ERR_FAIL_NULL_V(p_body, Variant());

	const JPH::SoftBodyMotionProperties &motion_properties = static_cast<const JPH::SoftBodyMotionProperties &>(*p_body->GetMotionPropertiesUnchecked());
	const int rods = (int)motion_properties.GetSettings()->mRodStretchShearConstraints.size();

	// Flattened, not boxed: one allocation instead of N `Variant`s.
	PackedFloat32Array out;
	out.resize(rods * 4);
	float *write = out.ptrw();

	for (int i = 0; i < rods; ++i) {
		const JPH::Quat rotation = motion_properties.GetRodRotation((JPH::uint)i);
		write[i * 4 + 0] = rotation.GetX();
		write[i * 4 + 1] = rotation.GetY();
		write[i * 4 + 2] = rotation.GetZ();
		write[i * 4 + 3] = rotation.GetW();
	}

	return out;
}

const CapProperty ROD_PROPS[] = {
	{ KEY_JOINTS, Variant::PACKED_VECTOR3_ARRAY, false },
	{ KEY_COMPLIANCE, Variant::PACKED_FLOAT32_ARRAY, false },
	{ KEY_BEND, Variant::PACKED_FLOAT32_ARRAY, false },
	{ KEY_FIXED, Variant::INT, false },
	{ KEY_STATE, Variant::PACKED_FLOAT32_ARRAY, true },
};

} // namespace

const CapabilitySpec CAP_ROD = {
	/*prefix*/ "rod/",
	/*required_key*/ KEY_JOINTS,
	/*props*/ Span<CapProperty>(ROD_PROPS, 5),
	/*geometry*/ GeometryRole::REPLACE,
	/*validate*/ rod_validate,
	/*contribute*/ rod_contribute,
	/*pinned_vertices*/ rod_pinned_vertices,
	/*apply_body*/ rod_apply_body,
	/*derive*/ rod_derive,
	/*get*/ rod_get,
};
