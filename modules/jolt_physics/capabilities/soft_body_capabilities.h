/**************************************************************************/
/*  soft_body_capabilities.h                                              */
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

#include "soft_body_capability.h"

#include "core/string/string_name.h"
#include "core/templates/span.h"

extern const CapabilitySpec CAP_ROD;

#ifdef TESTS_ENABLED
// A second geometry-replacing capability, without which the `replace_count > 1`
// arbitration is dead code no test can redden. Present only in a `tests=yes`
// build, so a shipping build has one capability and one only.
extern const CapabilitySpec CAP_TEST_REPLACE;

#include "cap_files.gen.h"
#endif

// The one list. `find_capability` and `all_capabilities` read the same array;
// a second hand-maintained list is the failure mode this shape prevents. The
// specs have external linkage so a static archive cannot discard their objects.
inline Span<const CapabilitySpec *const> all_capabilities() {
	static const CapabilitySpec *const kAll[] = {
		&CAP_ROD,
#ifdef TESTS_ENABLED
		&CAP_TEST_REPLACE,
#endif
	};
	return kAll;
}

// Resolves a fully-qualified key such as `rod/joints` to its owning capability,
// or `nullptr` when no capability claims the prefix.
inline const CapabilitySpec *find_capability(const StringName &p_name) {
	const String name = String(p_name);
	for (const CapabilitySpec *cap : all_capabilities()) {
		if (name.begins_with(cap->prefix)) {
			return cap;
		}
	}
	return nullptr;
}

// A capability is active when its `required_key` is present. Presence, not
// content: an empty required array still activates, which is why `validate`
// runs at set time and refuses one.
void cap_collect_active(const JoltSoftBodyCapState &p_state, LocalVector<const CapabilitySpec *> &r_active);

int cap_replace_count(const LocalVector<const CapabilitySpec *> &p_active);

// contribute -> derive -> Optimize(). The `Optimize()` call is the framework's,
// unconditional and exactly once: it is a global reordering, so two capabilities
// each calling it would corrupt each other's work, and there is deliberately no
// capability-author entry point for it.
void cap_build_settings(const JoltSoftBodyCapState &p_state, const LocalVector<const CapabilitySpec *> &p_active, JPH::SoftBodySharedSettings &r_settings, const LocalVector<int> &p_mesh_to_physics, float p_mass);

// Scalars land on a different Jolt object than the arrays, and `mUpdatePosition`
// must be written on every rebuild -- hence a second hook, called from
// `_add_to_space()`.
void cap_apply_body(const JoltSoftBodyCapState &p_state, const LocalVector<const CapabilitySpec *> &p_active, JPH::SoftBodyCreationSettings &r_settings);

// Re-applies every active capability's pinning. Templated because the settings
// path and the motion-properties path carry different vertex types, both with an
// `mInvMass` field.
template <typename TJoltVertex>
void cap_pin_vertices(const JoltSoftBodyCapState &p_state, JPH::Array<TJoltVertex> &r_vertices) {
	LocalVector<const CapabilitySpec *> active;
	cap_collect_active(p_state, active);

	LocalVector<int> indices;
	for (const CapabilitySpec *cap : active) {
		if (cap->pinned_vertices == nullptr) {
			continue;
		}
		indices.clear();
		cap->pinned_vertices(p_state, indices);
		for (int index : indices) {
			if (index >= 0 && index < (int)r_vertices.size()) {
				r_vertices[index].mInvMass = 0.0f;
			}
		}
	}
}
