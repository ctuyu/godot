/**************************************************************************/
/*  soft_body_cap_probe.h                                                 */
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

#include "core/math/vector3.h"
#include "core/string/ustring.h"
#include "core/templates/rid.h"

// Reading Jolt's soft-body state from a test case would force a Jolt header into
// the test translation unit, and the thirdparty include path lives on `env_jolt`
// alone -- adding it to the tests env would compile JPH without
// `JPH_DEBUG_RENDERER` and silently mismatch its layout against the module's.
// So every reading crosses this seam: POD in, POD out, bodies in the `.cpp`
// that `env_jolt` compiles.
namespace JoltSoftBodyCapProbe {

// True when the RID resolves to a Jolt soft body that is in a space.
bool in_space(RID p_body);

int vertex_count(RID p_body);
float vertex_inv_mass(RID p_body, int p_index);
Vector3 vertex_position(RID p_body, int p_index);
Vector3 center_of_mass(RID p_body);

int face_count(RID p_body);
int edge_count(RID p_body);

int rod_count(RID p_body);
int rod_bend_twist_count(RID p_body);
// `mRodStates` has no size accessor; the array is sized from the constraint
// count, so this reads the last state back to prove it is genuinely allocated.
bool rod_state_is_readable(RID p_body, int p_index);
float rod_length(RID p_body, int p_index);
float min_rod_length(RID p_body);
bool all_bishop_frames_set(RID p_body);
bool rod_angular_velocities_are_zero(RID p_body);
// `p_which` is 0 or 1.
int rod_vertex(RID p_body, int p_rod, int p_which);
int bend_twist_rod(RID p_body, int p_bend, int p_which);
float rod_compliance(RID p_body, int p_index);
float bend_twist_compliance(RID p_body, int p_index);

bool update_position(RID p_body);

// True when `p_server` IS the Jolt backend itself rather than something wrapping
// it. Every Jolt server is wrapped, so a test that got its server from
// `PhysicsServer3DManager` holds the wrapper and this must be false.
bool is_inner_jolt_server(const void *p_server);

// The registry itself. `soft_body_capabilities.h` pulls in Jolt headers, so even
// reading a `prefix` has to cross the seam.
int capability_count();
String capability_prefix(int p_cap);
String capability_required_key(int p_cap);
int capability_property_count(int p_cap);
String capability_property_name(int p_cap, int p_prop);
// The `Variant::Type` as an int, so the seam stays free of engine enums too.
int capability_property_type(int p_cap, int p_prop);
bool capability_property_live_read(int p_cap, int p_prop);

// The build-time list of `capabilities/cap_*.cpp`, basenames only.
int capability_source_file_count();
String capability_source_file(int p_index);

// Body-less checks over a freshly built `SoftBodySharedSettings`: they need no
// space, no server and no RID, which is what makes their rows unit rows.
bool calculate_rod_properties_keeps_chain_order(int p_joints);
bool optimize_is_identity_for_chain(int p_joints);

} // namespace JoltSoftBodyCapProbe
