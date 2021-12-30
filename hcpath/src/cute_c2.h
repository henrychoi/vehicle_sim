#pragma once

// this can be adjusted as necessary, but is highly recommended to be kept at 8.
// higher numbers will incur quite a bit of memory overhead, and convex shapes
// over 8 verts start to just look like spheres, which can be implicitly rep-
// resented as a point + radius. usually tools that generate polygons should be
// constructed so they do not output polygons with too many verts.
// Note: polygons in cute_c2 are all *convex*.
#define C2_MAX_POLYGON_VERTS 8

// 2d vector
typedef struct c2v
{
	float x;
	float y;
} c2v;

// 2d rotation composed of cos/sin pair
typedef struct c2r
{
	float c;
	float s;
} c2r;

// 2d rotation matrix
typedef struct c2m
{
	c2v x;
	c2v y;
} c2m;

// 2d transformation "x"
// These are used especially for c2Poly when a c2Poly is passed to a function.
// Since polygons are prime for "instancing" a c2x transform can be used to
// transform a polygon from local space to world space. In functions that take
// a c2x pointer (like c2PolytoPoly), these pointers can be NULL, which represents
// an identity transformation and assumes the verts inside of c2Poly are already
// in world space.
typedef struct c2x
{
	c2v p;
	c2r r;
} c2x;

// 2d halfspace (aka plane, aka line)
typedef struct c2h
{
	c2v n;   // normal, normalized
	float d; // distance to origin from plane, or ax + by = d
} c2h;

typedef struct c2Circle
{
	c2v p;
	float r;
} c2Circle;

typedef struct c2AABB
{
	c2v min;
	c2v max;
} c2AABB;

// a capsule is defined as a line segment (from a to b) and radius r
typedef struct c2Capsule
{
	c2v a;
	c2v b;
	float r;
} c2Capsule;

typedef struct c2Poly
{
	int count;
	c2v verts[C2_MAX_POLYGON_VERTS];
	c2v norms[C2_MAX_POLYGON_VERTS];
} c2Poly;

// IMPORTANT:
// Many algorithms in this file are sensitive to the magnitude of the
// ray direction (c2Ray::d). It is highly recommended to normalize the
// ray direction and use t to specify a distance. Please see this link
// for an in-depth explanation: https://github.com/RandyGaul/cute_headers/issues/30
typedef struct c2Ray
{
	c2v p;   // position
	c2v d;   // direction (normalized)
	float t; // distance along d from position p to find endpoint of ray
} c2Ray;

typedef struct c2Raycast
{
	float t; // time of impact
	c2v n;   // normal of surface at impact (unit length)
} c2Raycast;

typedef enum
{
	C2_TYPE_NONE,
	C2_TYPE_CIRCLE,
	C2_TYPE_AABB,
	C2_TYPE_CAPSULE,
	C2_TYPE_POLY
} C2_TYPE;

// This struct is only for advanced usage of the c2GJK function. See comments inside of the
// c2GJK function for more details.
typedef struct c2GJKCache
{
	float metric;
	int count;
	int iA[3];
	int iB[3];
	float div;
} c2GJKCache;

// This is an advanced function, intended to be used by people who know what they're doing.
//
// Runs the GJK algorithm to find closest points, returns distance between closest points.
// outA and outB can be NULL, in this case only distance is returned. ax_ptr and bx_ptr
// can be NULL, and represent local to world transformations for shapes A and B respectively.
// use_radius will apply radii for capsules and circles (if set to false, spheres are
// treated as points and capsules are treated as line segments i.e. rays). The cache parameter
// should be NULL, as it is only for advanced usage (unless you know what you're doing, then
// go ahead and use it). iterations is an optional parameter.
//
// IMPORTANT NOTE:
// The GJK function is sensitive to large shapes, since it internally will compute signed area
// values. `c2GJK` is called throughout cute c2 in many ways, so try to make sure all of your
// collision shapes are not gigantic. For example, try to keep the volume of all your shapes
// less than 100.0f. If you need large shapes, you should use tiny collision geometry for all
// cute c2 function, and simply render the geometry larger on-screen by scaling it up.
float c2GJK(const void* A, C2_TYPE typeA, const c2x* ax_ptr
	, const void* B, C2_TYPE typeB, const c2x* bx_ptr
	, c2v* outA, c2v* outB, bool use_radius
	, int* iterations, c2GJKCache* cache);

// Inflating a shape.
//
// This is useful to numerically grow or shrink a polytope. For example, when calling
// a time of impact function it can be good to use a slightly smaller shape. Then, once
// both shapes are moved to the time of impact a collision manifold can be made from the
// slightly larger (and now overlapping) shapes.
//
// IMPORTANT NOTE
// Inflating a shape with sharp corners can cause those corners to move dramatically.
// Deflating a shape can avoid this problem, but deflating a very small shape can invert
// the planes and result in something that is no longer convex. Make sure to pick an
// appropriately small skin factor, for example 1.0e-6f.
void c2Inflate(void* shape, C2_TYPE type, float skin_factor);

// Computes 2D convex hull. Will not do anything if less than two verts supplied. If
// more than C2_MAX_POLYGON_VERTS are supplied extras are ignored.
int c2Hull(c2v* verts, int count);
void c2Norms(c2v* verts, c2v* norms, int count);

// runs c2Hull and c2Norms, assumes p->verts and p->count are both set to valid values
void c2MakePoly(c2Poly* p);

/*
	------------------------------------------------------------------------------
	This software is available under 2 licenses - you may choose the one you like.
	------------------------------------------------------------------------------
	ALTERNATIVE A - zlib license
	Copyright (c) 2017 Randy Gaul http://www.randygaul.net
	This software is provided 'as-is', without any express or implied warranty.
	In no event will the authors be held liable for any damages arising from
	the use of this software.
	Permission is granted to anyone to use this software for any purpose,
	including commercial applications, and to alter it and redistribute it
	freely, subject to the following restrictions:
	  1. The origin of this software must not be misrepresented; you must not
	     claim that you wrote the original software. If you use this software
	     in a product, an acknowledgment in the product documentation would be
	     appreciated but is not required.
	  2. Altered source versions must be plainly marked as such, and must not
	     be misrepresented as being the original software.
	  3. This notice may not be removed or altered from any source distribution.
	------------------------------------------------------------------------------
	ALTERNATIVE B - Public Domain (www.unlicense.org)
	This is free and unencumbered software released into the public domain.
	Anyone is free to copy, modify, publish, use, compile, sell, or distribute this 
	software, either in source code form or as a compiled binary, for any purpose, 
	commercial or non-commercial, and by any means.
	In jurisdictions that recognize copyright laws, the author or authors of this 
	software dedicate any and all copyright interest in the software to the public 
	domain. We make this dedication for the benefit of the public at large and to 
	the detriment of our heirs and successors. We intend this dedication to be an 
	overt act of relinquishment in perpetuity of all present and future rights to 
	this software under copyright law.
	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
	AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN 
	ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION 
	WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
	------------------------------------------------------------------------------
*/
