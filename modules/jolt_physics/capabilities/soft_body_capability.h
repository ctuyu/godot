/**************************************************************************/
/*  soft_body_capability.h                                                */
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

#include "core/string/string_name.h"
#include "core/templates/local_vector.h"
#include "core/templates/span.h"
#include "core/variant/variant.h"

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>

class JoltSoftBodyCapState;

// Where a capability's vertices come from. `REPLACE` supplies them itself and
// ignores the mesh; `NONE` relies on the mesh path, so any index it holds must
// be translated through `mesh_to_physics`. `EXTEND` is declared for future use.
enum class GeometryRole {
	NONE,
	EXTEND,
	REPLACE,
};

// `live_read` does double duty: it clears PROPERTY_USAGE_STORAGE in the property
// list, and it selects the `in_space()`-guarded arm of the dispatcher. A writable
// key carries STORAGE, so a serialiser will read it on a body that has no space.
struct CapProperty {
	const char *name;
	Variant::Type type;
	bool live_read;
};

struct CapabilitySpec {
	const char *prefix;

	// Presence of this key is what makes the capability active. Not "any key
	// under the prefix": keys arrive as separate calls, so a partially
	// configured capability must be a representable, harmless state.
	const char *required_key;

	Span<CapProperty> props;
	GeometryRole geometry;

	// Validates every key present plus every cross-key relation whose operands
	// are both present. Runs at set time, over the post-write state.
	bool (*validate)(const JoltSoftBodyCapState &p_state, String *r_err);

	// `p_mass` is the body's total mass; a capability supplying vertices must
	// write the same inverse mass `_update_mass()` will write, or the settings
	// and the body disagree one step later.
	void (*contribute)(const JoltSoftBodyCapState &p_state, JPH::SoftBodySharedSettings &r_settings, const LocalVector<int> &p_mesh_to_physics, float p_mass);

	// The physics-vertex indices this capability pins (`mInvMass = 0`). One
	// source of truth, read both by `contribute` and by the re-pin that follows
	// `_update_mass()` — which rewrites every vertex's inverse mass and would
	// otherwise wipe them.
	void (*pinned_vertices)(const JoltSoftBodyCapState &p_state, LocalVector<int> &r_indices);

	void (*apply_body)(const JoltSoftBodyCapState &p_state, JPH::SoftBodyCreationSettings &r_settings);
	void (*derive)(JPH::SoftBodySharedSettings &r_settings);

	// The body is passed by reference, so this cannot express "there is no
	// body"; the dispatcher runs the `in_space()` guard before calling.
	// `p_body` is non-null exactly when the requested property is `live_read`;
	// the dispatcher owns that guard, because a capability cannot see whether the
	// body is in a space. A stored key is served with no body at all, so a
	// serialiser can read it outside a space.
	Variant (*get)(const JoltSoftBodyCapState &p_state, const JPH::Body *p_body, const StringName &p_name);
};
