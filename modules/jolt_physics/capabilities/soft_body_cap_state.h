/**************************************************************************/
/*  soft_body_cap_state.h                                                 */
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
#include "core/templates/hash_map.h"
#include "core/variant/variant.h"

// Key/value storage for one soft body's capability configuration.
//
// Lifecycle, all three arms decided deliberately:
//  - `set_mesh` does NOT clear this; geometry source and capability
//    configuration are independent.
//  - a space change does NOT clear this; leaving one space and entering another
//    must rebuild from the same configuration.
//  - `soft_body_create()` DOES clear it, or RID reuse leaks one body's
//    configuration into the next.
class JoltSoftBodyCapState {
	HashMap<StringName, Variant> values;

public:
	bool has(const StringName &p_name) const { return values.has(p_name); }

	Variant get(const StringName &p_name) const {
		const Variant *value = values.getptr(p_name);
		return value != nullptr ? *value : Variant();
	}

	void set(const StringName &p_name, const Variant &p_value) { values[p_name] = p_value; }

	void clear() { values.clear(); }

	PackedVector3Array v3(const StringName &p_name) const {
		const Variant *value = values.getptr(p_name);
		return value != nullptr ? PackedVector3Array(*value) : PackedVector3Array();
	}

	PackedFloat32Array f32(const StringName &p_name) const {
		const Variant *value = values.getptr(p_name);
		return value != nullptr ? PackedFloat32Array(*value) : PackedFloat32Array();
	}

	int i(const StringName &p_name, int p_default = 0) const {
		const Variant *value = values.getptr(p_name);
		return value != nullptr ? (int)*value : p_default;
	}
};

// The three-way outcome of a write, per the capability channel's contract:
// a known-but-invalid write is an error naming the actual numbers, while an
// unknown prefix is silent so a caller can probe a backend and fall back.
enum class CapStoreResult {
	OK,
	UNKNOWN_PREFIX,
	REJECTED,
};

// Type-checks the write, runs the owning capability's `validate` over a scratch
// copy of the post-write state, and commits only on success. A rejected write
// leaves `r_state` exactly as it was.
CapStoreResult cap_state_store(JoltSoftBodyCapState &r_state, const StringName &p_name, const Variant &p_value, String *r_err);
