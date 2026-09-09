/**************************************************************************/
/*  soft_body_capability.cpp                                              */
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

#include "soft_body_capability.h"

#include "soft_body_cap_state.h"
#include "soft_body_capabilities.h"

void cap_collect_active(const JoltSoftBodyCapState &p_state, LocalVector<const CapabilitySpec *> &r_active) {
	r_active.clear();

	for (const CapabilitySpec *cap : all_capabilities()) {
		if (p_state.has(StringName(cap->required_key))) {
			r_active.push_back(cap);
		}
	}
}

int cap_replace_count(const LocalVector<const CapabilitySpec *> &p_active) {
	int count = 0;

	for (const CapabilitySpec *cap : p_active) {
		if (cap->geometry == GeometryRole::REPLACE) {
			++count;
		}
	}

	return count;
}

void cap_build_settings(const JoltSoftBodyCapState &p_state, const LocalVector<const CapabilitySpec *> &p_active, JPH::SoftBodySharedSettings &r_settings, const LocalVector<int> &p_mesh_to_physics, float p_mass) {
	for (const CapabilitySpec *cap : p_active) {
		if (cap->contribute != nullptr) {
			cap->contribute(p_state, r_settings, p_mesh_to_physics, p_mass);
		}
	}

	for (const CapabilitySpec *cap : p_active) {
		if (cap->derive != nullptr) {
			cap->derive(r_settings);
		}
	}

	r_settings.Optimize();
}

void cap_apply_body(const JoltSoftBodyCapState &p_state, const LocalVector<const CapabilitySpec *> &p_active, JPH::SoftBodyCreationSettings &r_settings) {
	for (const CapabilitySpec *cap : p_active) {
		if (cap->apply_body != nullptr) {
			cap->apply_body(p_state, r_settings);
		}
	}
}

CapStoreResult cap_state_store(JoltSoftBodyCapState &r_state, const StringName &p_name, const Variant &p_value, String *r_err) {
	const CapabilitySpec *cap = find_capability(p_name);
	if (cap == nullptr) {
		return CapStoreResult::UNKNOWN_PREFIX;
	}

	const CapProperty *prop = nullptr;
	for (const CapProperty &candidate : cap->props) {
		if (p_name == StringName(candidate.name)) {
			prop = &candidate;
			break;
		}
	}

	if (prop == nullptr) {
		*r_err = vformat("'%s' is not a valid key of the '%s' capability.", String(p_name), cap->prefix);
		return CapStoreResult::REJECTED;
	}

	if (prop->live_read) {
		*r_err = vformat("'%s' is read-only and cannot be written.", String(p_name));
		return CapStoreResult::REJECTED;
	}

	if (p_value.get_type() != prop->type) {
		*r_err = vformat("'%s' expects a value of type %s, got %s.", String(p_name), Variant::get_type_name(prop->type), Variant::get_type_name(p_value.get_type()));
		return CapStoreResult::REJECTED;
	}

	// Validate the whole prefix's post-write state on a scratch copy, so a
	// rejected write leaves the caller's state exactly as it was. Per-key
	// validation would be order-dependent (`rod/fixed`'s bound needs
	// `rod/joints`) and would miss a shrink that strands a sibling array.
	JoltSoftBodyCapState scratch = r_state;
	scratch.set(p_name, p_value);

	if (cap->validate != nullptr && !cap->validate(scratch, r_err)) {
		return CapStoreResult::REJECTED;
	}

	r_state = scratch;
	return CapStoreResult::OK;
}
