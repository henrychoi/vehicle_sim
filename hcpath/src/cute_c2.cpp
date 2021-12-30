#include "cute_c2.h"
#define C2_INLINE inline __attribute__((always_inline))

// adjust these primitives as seen fit
#include <string.h> // memcpy
#include <math.h>
#define c2Sin(radians) sinf(radians)
#define c2Cos(radians) cosf(radians)
#define c2Sqrt(a) sqrtf(a)
#define c2Min(a, b) ((a) < (b) ? (a) : (b))
#define c2Max(a, b) ((a) > (b) ? (a) : (b))
#define c2Abs(a) ((a) < 0 ? -(a) : (a))
#define c2Clamp(a, lo, hi) c2Max(lo, c2Min(a, hi))
C2_INLINE void c2SinCos(float radians, float* s, float* c) { *c = c2Cos(radians); *s = c2Sin(radians); }
#define c2Sign(a) (a < 0 ? -1.0f : 1.0f)

// The rest of the functions in the header-only portion are all for internal use
// and use the author's personal naming conventions. It is recommended to use one's
// own math library instead of the one embedded here in cute_c2, but for those
// curious or interested in trying it out here's the details:

// The Mul functions are used to perform multiplication. x stands for transform,
// v stands for vector, s stands for scalar, r stands for rotation, h stands for
// halfspace and T stands for transpose.For example c2MulxvT stands for "multiply
// a transform with a vector, and transpose the transform".

// vector ops
C2_INLINE c2v c2V(float x, float y) { c2v a; a.x = x; a.y = y; return a; }
C2_INLINE c2v c2Add(c2v a, c2v b) { a.x += b.x; a.y += b.y; return a; }
C2_INLINE c2v c2Sub(c2v a, c2v b) { a.x -= b.x; a.y -= b.y; return a; }
C2_INLINE float c2Dot(c2v a, c2v b) { return a.x * b.x + a.y * b.y; }
C2_INLINE c2v c2Mulvs(c2v a, float b) { a.x *= b; a.y *= b; return a; }
C2_INLINE c2v c2Mulvv(c2v a, c2v b) { a.x *= b.x; a.y *= b.y; return a; }
C2_INLINE c2v c2Div(c2v a, float b) { return c2Mulvs(a, 1.0f / b); }
C2_INLINE c2v c2Skew(c2v a) { c2v b; b.x = -a.y; b.y = a.x; return b; }
C2_INLINE c2v c2CCW90(c2v a) { c2v b; b.x = a.y; b.y = -a.x; return b; }
C2_INLINE float c2Det2(c2v a, c2v b) { return a.x * b.y - a.y * b.x; }
C2_INLINE c2v c2Minv(c2v a, c2v b) { return c2V(c2Min(a.x, b.x), c2Min(a.y, b.y)); }
C2_INLINE c2v c2Maxv(c2v a, c2v b) { return c2V(c2Max(a.x, b.x), c2Max(a.y, b.y)); }
C2_INLINE c2v c2Clampv(c2v a, c2v lo, c2v hi) { return c2Maxv(lo, c2Minv(a, hi)); }
C2_INLINE c2v c2Absv(c2v a) { return c2V(c2Abs(a.x), c2Abs(a.y)); }
C2_INLINE float c2Hmin(c2v a) { return c2Min(a.x, a.y); }
C2_INLINE float c2Hmax(c2v a) { return c2Max(a.x, a.y); }
C2_INLINE float c2Len(c2v a) { return c2Sqrt(c2Dot(a, a)); }
C2_INLINE c2v c2Norm(c2v a) { return c2Div(a, c2Len(a)); }
C2_INLINE c2v c2SafeNorm(c2v a) { float sq = c2Dot(a, a); return sq ? c2Div(a, c2Len(a)) : c2V(0, 0); }
C2_INLINE c2v c2Neg(c2v a) { return c2V(-a.x, -a.y); }
C2_INLINE c2v c2Lerp(c2v a, c2v b, float t) { return c2Add(a, c2Mulvs(c2Sub(b, a), t)); }
C2_INLINE int c2Parallel(c2v a, c2v b, float kTol)
{
	float k = c2Len(a) / c2Len(b);
	b = c2Mulvs(b, k);
	if (c2Abs(a.x - b.x) < kTol && c2Abs(a.y - b.y) < kTol) return 1;
	return 0;
}

// rotation ops
C2_INLINE c2r c2Rot(float radians) { c2r r; c2SinCos(radians, &r.s, &r.c); return r; }
C2_INLINE c2r c2RotIdentity(void) { c2r r; r.c = 1.0f; r.s = 0; return r; }
C2_INLINE c2v c2RotX(c2r r) { return c2V(r.c, r.s); }
C2_INLINE c2v c2RotY(c2r r) { return c2V(-r.s, r.c); }
C2_INLINE c2v c2Mulrv(c2r a, c2v b)  { return c2V(a.c * b.x - a.s * b.y,  a.s * b.x + a.c * b.y); }
C2_INLINE c2v c2MulrvT(c2r a, c2v b) { return c2V(a.c * b.x + a.s * b.y, -a.s * b.x + a.c * b.y); }
C2_INLINE c2r c2Mulrr(c2r a, c2r b)  { c2r c; c.c = a.c * b.c - a.s * b.s; c.s = a.s * b.c + a.c * b.s; return c; }
C2_INLINE c2r c2MulrrT(c2r a, c2r b) { c2r c; c.c = a.c * b.c + a.s * b.s; c.s = a.c * b.s - a.s * b.c; return c; }

C2_INLINE c2v c2Mulmv(c2m a, c2v b) { c2v c; c.x = a.x.x * b.x + a.y.x * b.y; c.y = a.x.y * b.x + a.y.y * b.y; return c; }
C2_INLINE c2v c2MulmvT(c2m a, c2v b) { c2v c; c.x = a.x.x * b.x + a.x.y * b.y; c.y = a.y.x * b.x + a.y.y * b.y; return c; }
C2_INLINE c2m c2Mulmm(c2m a, c2m b)  { c2m c; c.x = c2Mulmv(a, b.x);  c.y = c2Mulmv(a, b.y); return c; }
C2_INLINE c2m c2MulmmT(c2m a, c2m b) { c2m c; c.x = c2MulmvT(a, b.x); c.y = c2MulmvT(a, b.y); return c; }

// transform ops
C2_INLINE c2x c2xIdentity(void) { c2x x; x.p = c2V(0, 0); x.r = c2RotIdentity(); return x; }
C2_INLINE c2v c2Mulxv(c2x a, c2v b) { return c2Add(c2Mulrv(a.r, b), a.p); }
C2_INLINE c2v c2MulxvT(c2x a, c2v b) { return c2MulrvT(a.r, c2Sub(b, a.p)); }
C2_INLINE c2x c2Mulxx(c2x a, c2x b) { c2x c; c.r = c2Mulrr(a.r, b.r); c.p = c2Add(c2Mulrv(a.r, b.p), a.p); return c; }
C2_INLINE c2x c2MulxxT(c2x a, c2x b) { c2x c; c.r = c2MulrrT(a.r, b.r); c.p = c2MulrvT(a.r, c2Sub(b.p, a.p)); return c; }
C2_INLINE c2x c2Transform(c2v p, float radians) { c2x x; x.r = c2Rot(radians); x.p = p; return x; }

// halfspace ops
C2_INLINE c2v c2Origin(c2h h) { return c2Mulvs(h.n, h.d); }
C2_INLINE float c2Dist(c2h h, c2v p) { return c2Dot(h.n, p) - h.d; }
C2_INLINE c2v c2Project(c2h h, c2v p) { return c2Sub(p, c2Mulvs(h.n, c2Dist(h, p))); }
C2_INLINE c2h c2Mulxh(c2x a, c2h b) { c2h c; c.n = c2Mulrv(a.r, b.n); c.d = c2Dot(c2Mulxv(a, c2Origin(b)), c.n); return c; }
C2_INLINE c2h c2MulxhT(c2x a, c2h b) { c2h c; c.n = c2MulrvT(a.r, b.n); c.d = c2Dot(c2MulxvT(a, c2Origin(b)), c.n); return c; }
C2_INLINE c2v c2Intersect(c2v a, c2v b, float da, float db) { return c2Add(a, c2Mulvs(c2Sub(b, a), (da / (da - db)))); }

C2_INLINE void c2BBVerts(c2v* out, c2AABB* bb)
{
	out[0] = bb->min;
	out[1] = c2V(bb->max.x, bb->min.y);
	out[2] = bb->max;
	out[3] = c2V(bb->min.x, bb->max.y);
}

#define C2_GJK_ITERS 20

typedef struct
{
	float radius;
	int count;
	c2v verts[C2_MAX_POLYGON_VERTS];
} c2Proxy;

typedef struct
{
	c2v sA;
	c2v sB;
	c2v p;
	float u;
	int iA;
	int iB;
} c2sv;

typedef struct
{
	c2sv a, b, c, d;
	float div;
	int count;
} c2Simplex;

C2_INLINE void c2MakeProxy(const void* shape, C2_TYPE type, c2Proxy* p)
{
	switch (type)
	{
	case C2_TYPE_CIRCLE:
	{
		c2Circle* c = (c2Circle*)shape;
		p->radius = c->r;
		p->count = 1;
		p->verts[0] = c->p;
	}	break;

	case C2_TYPE_AABB:
	{
		c2AABB* bb = (c2AABB*)shape;
		p->radius = 0;
		p->count = 4;
		c2BBVerts(p->verts, bb);
	}	break;

	case C2_TYPE_CAPSULE:
	{
		c2Capsule* c = (c2Capsule*)shape;
		p->radius = c->r;
		p->count = 2;
		p->verts[0] = c->a;
		p->verts[1] = c->b;
	}	break;

	case C2_TYPE_POLY:
	{
		c2Poly* poly = (c2Poly*)shape;
		p->radius = 0;
		p->count = poly->count;
		for (int i = 0; i < p->count; ++i) p->verts[i] = poly->verts[i];
	}	break;
	}
}

C2_INLINE int c2Support(const c2v* verts, int count, c2v d)
{
	int imax = 0;
	float dmax = c2Dot(verts[0], d);

	for (int i = 1; i < count; ++i)
	{
		float dot = c2Dot(verts[i], d);
		if (dot > dmax)
		{
			imax = i;
			dmax = dot;
		}
	}

	return imax;
}

#define C2_BARY(n, x) c2Mulvs(s->n.x, (den * s->n.u))
#define C2_BARY2(x) c2Add(C2_BARY(a, x), C2_BARY(b, x))
#define C2_BARY3(x) c2Add(c2Add(C2_BARY(a, x), C2_BARY(b, x)), C2_BARY(c, x))

C2_INLINE c2v c2L(c2Simplex* s)
{
	float den = 1.0f / s->div;
	switch (s->count)
	{
	case 1: return s->a.p;
	case 2: return C2_BARY2(p);
	case 3: return C2_BARY3(p);
	default: return c2V(0, 0);
	}
}

C2_INLINE void c2Witness(c2Simplex* s, c2v* a, c2v* b)
{
	float den = 1.0f / s->div;
	switch (s->count)
	{
	case 1: *a = s->a.sA; *b = s->a.sB; break;
	case 2: *a = C2_BARY2(sA); *b = C2_BARY2(sB); break;
	case 3: *a = C2_BARY3(sA); *b = C2_BARY3(sB); break;
	default: *a = c2V(0, 0); *b = c2V(0, 0);
	}
}

C2_INLINE c2v c2D(c2Simplex* s)
{
	switch (s->count)
	{
	case 1: return c2Neg(s->a.p);
	case 2:
	{
		c2v ab = c2Sub(s->b.p, s->a.p);
		if (c2Det2(ab, c2Neg(s->a.p)) > 0) return c2Skew(ab);
		return c2CCW90(ab);
	}
	case 3:
	default: return c2V(0, 0);
	}
}

C2_INLINE void c22(c2Simplex* s)
{
	c2v a = s->a.p;
	c2v b = s->b.p;
	float u = c2Dot(b, c2Sub(b, a));
	float v = c2Dot(a, c2Sub(a, b));

	if (v <= 0)
	{
		s->a.u = 1.0f;
		s->div = 1.0f;
		s->count = 1;
	}

	else if (u <= 0)
	{
		s->a = s->b;
		s->a.u = 1.0f;
		s->div = 1.0f;
		s->count = 1;
	}

	else
	{
		s->a.u = u;
		s->b.u = v;
		s->div = u + v;
		s->count = 2;
	}
}

C2_INLINE void c23(c2Simplex* s)
{
	c2v a = s->a.p;
	c2v b = s->b.p;
	c2v c = s->c.p;

	float uAB = c2Dot(b, c2Sub(b, a));
	float vAB = c2Dot(a, c2Sub(a, b));
	float uBC = c2Dot(c, c2Sub(c, b));
	float vBC = c2Dot(b, c2Sub(b, c));
	float uCA = c2Dot(a, c2Sub(a, c));
	float vCA = c2Dot(c, c2Sub(c, a));
	float area = c2Det2(c2Sub(b, a), c2Sub(c, a));
	float uABC = c2Det2(b, c) * area;
	float vABC = c2Det2(c, a) * area;
	float wABC = c2Det2(a, b) * area;

	if (vAB <= 0 && uCA <= 0)
	{
		s->a.u = 1.0f;
		s->div = 1.0f;
		s->count = 1;
	}

	else if (uAB <= 0 && vBC <= 0)
	{
		s->a = s->b;
		s->a.u = 1.0f;
		s->div = 1.0f;
		s->count = 1;
	}

	else if (uBC <= 0 && vCA <= 0)
	{
		s->a = s->c;
		s->a.u = 1.0f;
		s->div = 1.0f;
		s->count = 1;
	}

	else if (uAB > 0 && vAB > 0 && wABC <= 0)
	{
		s->a.u = uAB;
		s->b.u = vAB;
		s->div = uAB + vAB;
		s->count = 2;
	}

	else if (uBC > 0 && vBC > 0 && uABC <= 0)
	{
		s->a = s->b;
		s->b = s->c;
		s->a.u = uBC;
		s->b.u = vBC;
		s->div = uBC + vBC;
		s->count = 2;
	}

	else if (uCA > 0 && vCA > 0 && vABC <= 0)
	{
		s->b = s->a;
		s->a = s->c;
		s->a.u = uCA;
		s->b.u = vCA;
		s->div = uCA + vCA;
		s->count = 2;
	}

	else
	{
		s->a.u = uABC;
		s->b.u = vABC;
		s->c.u = wABC;
		s->div = uABC + vABC + wABC;
		s->count = 3;
	}
}

#include <float.h>

C2_INLINE float c2GJKSimplexMetric(c2Simplex* s)
{
	switch (s->count)
	{
	default: // fall through
	case 1:  return 0;
	case 2:  return c2Len(c2Sub(s->b.p, s->a.p));
	case 3:  return c2Det2(c2Sub(s->b.p, s->a.p), c2Sub(s->c.p, s->a.p));
	}
}

C2_INLINE float c2Step(float t, const void* A, C2_TYPE typeA, const c2x* ax_ptr, c2v vA, c2v* a, const void* B, C2_TYPE typeB, const c2x* bx_ptr, c2v vB, c2v* b, int use_radius, c2GJKCache* cache)
{
	c2x ax = *ax_ptr;
	c2x bx = *bx_ptr;
	ax.p = c2Add(ax.p, c2Mulvs(vA, t));
	bx.p = c2Add(bx.p, c2Mulvs(vB, t));
	float d = c2GJK(A, typeA, &ax, B, typeB, &bx, a, b, use_radius, NULL, cache);
	return d;
}

C2_INLINE c2h c2PlaneAt(const c2Poly* p, const int i)
{
	c2h h;
	h.n = p->norms[i];
	h.d = c2Dot(p->norms[i], p->verts[i]);
	return h;
}

C2_INLINE c2v c2CapsuleSupport(c2Capsule A, c2v dir)
{
	float da = c2Dot(A.a, dir);
	float db = c2Dot(A.b, dir);
	if (da > db) return c2Add(A.a, c2Mulvs(dir, A.r));
	else return c2Add(A.b, c2Mulvs(dir, A.r));
}

// position of impact p = ray.p + ray.d * raycast.t
#define c2Impact(ray, t) c2Add(ray.p, c2Mulvs(ray.d, t))


// Please see http://box2d.org/downloads/ under GDC 2010 for Erin's demo code
// and PDF slides for documentation on the GJK algorithm. This function is mostly
// from Erin's version from his online resources.
// @param ax_ptr 2D transform for A
// @param outA In case of no collision, the point on A closest to B
// @param use_radius  In case of no collision, use radius to estimate the distance
// @param iterations Indicates a problem if iterations blows up (e.g. numeric)
// @param cache InOut algo will speed up if cache is used
// @return distance between the A and B convex shapes
float c2GJK(const void* A, C2_TYPE typeA, const c2x* ax_ptr
		, const void* B, C2_TYPE typeB, const c2x* bx_ptr
		, c2v* outA, c2v* outB, bool use_radius
		, int* iterations, c2GJKCache* cache)
{
	c2x ax;
	c2x bx;
	if (!ax_ptr) ax = c2xIdentity();
	else ax = *ax_ptr;
	if (!bx_ptr) bx = c2xIdentity();
	else bx = *bx_ptr;

	c2Proxy pA;
	c2Proxy pB;
	c2MakeProxy(A, typeA, &pA);
	c2MakeProxy(B, typeB, &pB);

	c2Simplex s;
	c2sv* verts = &s.a;

	// Metric and caching system as designed by E. Catto in Box2D for his conservative advancment/bilateral
	// advancement algorithim implementations. The purpose is to reuse old simplex indices (any simplex that
	// have not degenerated into a line or point) as a starting point. This skips the first few iterations of
	// GJK going from point, to line, to triangle, lowering convergence rates dramatically for temporally
	// coherent cases (such as in time of impact searches).
	int cache_was_read = 0;
	if (cache)
	{
		int cache_was_good = !!cache->count;

		if (cache_was_good)
		{
			for (int i = 0; i < cache->count; ++i)
			{
				int iA = cache->iA[i];
				int iB = cache->iB[i];
				c2v sA = c2Mulxv(ax, pA.verts[iA]);
				c2v sB = c2Mulxv(bx, pB.verts[iB]);
				c2sv* v = verts + i;
				v->iA = iA;
				v->sA = sA;
				v->iB = iB;
				v->sB = sB;
				v->p = c2Sub(v->sB, v->sA);
				v->u = 0;
			}
			s.count = cache->count;
			s.div = cache->div;

			float metric_old = cache->metric;
			float metric = c2GJKSimplexMetric(&s);

			float min_metric = metric < metric_old ? metric : metric_old;
			float max_metric = metric > metric_old ? metric : metric_old;

			if (!(min_metric < max_metric * 2.0f && metric < -1.0e8f)) cache_was_read = 1;
		}
	}

	if (!cache_was_read)
	{
		s.a.iA = 0;
		s.a.iB = 0;
		s.a.sA = c2Mulxv(ax, pA.verts[0]);
		s.a.sB = c2Mulxv(bx, pB.verts[0]);
		s.a.p = c2Sub(s.a.sB, s.a.sA);
		s.a.u = 1.0f;
		s.div = 1.0f;
		s.count = 1;
	}

	int saveA[3], saveB[3];
	int save_count = 0;
	float d0 = FLT_MAX;
	float d1 = FLT_MAX;
	int iter = 0;
	int hit = 0;
	while (iter < C2_GJK_ITERS)
	{
		save_count = s.count;
		for (int i = 0; i < save_count; ++i)
		{
			saveA[i] = verts[i].iA;
			saveB[i] = verts[i].iB;
		}
		
		switch (s.count)
		{
		case 1: break;
		case 2: c22(&s); break;
		case 3: c23(&s); break;
		}

		if (s.count == 3)
		{
			hit = 1;
			break;
		}

		c2v p = c2L(&s);
		d1 = c2Dot(p, p);

		if (d1 > d0) break;
		d0 = d1;

		c2v d = c2D(&s);
		if (c2Dot(d, d) < FLT_EPSILON * FLT_EPSILON) break;

		int iA = c2Support(pA.verts, pA.count, c2MulrvT(ax.r, c2Neg(d)));
		c2v sA = c2Mulxv(ax, pA.verts[iA]);
		int iB = c2Support(pB.verts, pB.count, c2MulrvT(bx.r, d));
		c2v sB = c2Mulxv(bx, pB.verts[iB]);

		c2sv* v = verts + s.count;
		v->iA = iA;
		v->sA = sA;
		v->iB = iB;
		v->sB = sB;
		v->p = c2Sub(v->sB, v->sA);

		int dup = 0;
		for (int i = 0; i < save_count; ++i)
		{
			if (iA == saveA[i] && iB == saveB[i])
			{
				dup = 1;
				break;
			}
		}
		if (dup) break;

		++s.count;
		++iter;
	}

	c2v a, b;
	c2Witness(&s, &a, &b);
	float dist = c2Len(c2Sub(a, b));

	if (hit)
	{
		a = b;
		dist = 0;
	}

	else if (use_radius)
	{
		float rA = pA.radius;
		float rB = pB.radius;

		if (dist > rA + rB && dist > FLT_EPSILON)
		{
			dist -= rA + rB;
			c2v n = c2Norm(c2Sub(b, a));
			a = c2Add(a, c2Mulvs(n, rA));
			b = c2Sub(b, c2Mulvs(n, rB));
			if (a.x == b.x && a.y == b.y) dist = 0;
		}

		else
		{
			c2v p = c2Mulvs(c2Add(a, b), 0.5f);
			a = p;
			b = p;
			dist = 0;
		}
	}

	if (cache)
	{
		cache->metric = c2GJKSimplexMetric(&s);
		cache->count = s.count;
		for (int i = 0; i < s.count; ++i)
		{
			c2sv* v = verts + i;
			cache->iA[i] = v->iA;
			cache->iB[i] = v->iB;
		}
		cache->div = s.div;
	}

	if (outA) *outA = a;
	if (outB) *outB = b;
	if (iterations) *iterations = iter;
	return dist;
}

float c2TOI(const void* A, C2_TYPE typeA, const c2x* ax_ptr, c2v vA, const void* B, C2_TYPE typeB, const c2x* bx_ptr, c2v vB, int use_radius, int* iterations)
{
	float t = 0;
	c2x ax;
	c2x bx;
	if (!ax_ptr) ax = c2xIdentity();
	else ax = *ax_ptr;
	if (!bx_ptr) bx = c2xIdentity();
	else bx = *bx_ptr;
	c2v a, b;
	c2GJKCache cache;
	cache.count = 0;
	float d = c2Step(t, A, typeA, &ax, vA, &a, B, typeB, &bx, vB, &b, use_radius, &cache);
	c2v v = c2Sub(vB, vA);

	int iters = 0;
	float eps = 1.0e-6f;
	while (d > eps && t < 1)
	{
		++iters;
		float velocity_bound = c2Abs(c2Dot(c2Norm(c2Sub(b, a)), v));
		if (!velocity_bound) return 1;
		float delta = d / velocity_bound;
		float t0 = t;
		float t1 = t + delta;
		if (t0 == t1) break;
		t = t1;
		d = c2Step(t, A, typeA, &ax, vA, &a, B, typeB, &bx, vB, &b, use_radius, &cache);
	}

	t = t >= 1 ? 1 : t;
	if (iterations) *iterations = iters;

	return t;
}

int c2Hull(c2v* verts, int count)
{
	if (count <= 2) return 0;
	count = c2Min(C2_MAX_POLYGON_VERTS, count);

	int right = 0;
	float xmax = verts[0].x;
	for (int i = 1; i < count; ++i)
	{
		float x = verts[i].x;
		if (x > xmax)
		{
			xmax = x;
			right = i;
		}

		else if (x == xmax)
		if (verts[i].y < verts[right].y) right = i;
	}

	int hull[C2_MAX_POLYGON_VERTS];
	int out_count = 0;
	int index = right;

	while (1)
	{
		hull[out_count] = index;
		int next = 0;

		for (int i = 1; i < count; ++i)
		{
			if (next == index)
			{
				next = i;
				continue;
			}

			c2v e1 = c2Sub(verts[next], verts[hull[out_count]]);
			c2v e2 = c2Sub(verts[i], verts[hull[out_count]]);
			float c = c2Det2(e1, e2);
			if(c < 0) next = i;
			if (c == 0 && c2Dot(e2, e2) > c2Dot(e1, e1)) next = i;
		}

		++out_count;
		index = next;
		if (next == right) break;
	}

	c2v hull_verts[C2_MAX_POLYGON_VERTS];
	for (int i = 0; i < out_count; ++i) hull_verts[i] = verts[hull[i]];
	memcpy(verts, hull_verts, sizeof(c2v) * out_count);
	return out_count;
}

void c2Norms(c2v* verts, c2v* norms, int count)
{
	for (int  i = 0; i < count; ++i)
	{
		int a = i;
		int b = i + 1 < count ? i + 1 : 0;
		c2v e = c2Sub(verts[b], verts[a]);
		norms[i] = c2Norm(c2CCW90(e));
	}
}

void c2MakePoly(c2Poly* p)
{
	p->count = c2Hull(p->verts, p->count);
	c2Norms(p->verts, p->norms, p->count);
}

c2Poly c2Dual(c2Poly poly, float skin_factor)
{
	c2Poly dual;
	dual.count = poly.count;

	// Each plane maps to a point by involution (the mapping is its own inverse) by dividing
	// the plane normal by its offset factor.
	// plane = a * x + b * y - d
	// dual = { a / d, b / d }
	for (int i = 0; i < poly.count; ++i) {
		c2v n = poly.norms[i];
		float d = c2Dot(n, poly.verts[i]) - skin_factor;
		if (d == 0) dual.verts[i] = c2V(0, 0);
		else dual.verts[i] = c2Div(n, d);
	}

	// Instead of canonically building the convex hull, can simply take advantage of how
	// the vertices are still in proper CCW order, so only the normals must be recomputed.
	c2Norms(dual.verts, dual.norms, dual.count);

	return dual;
}

// Inflating a polytope, idea by Dirk Gregorius ~ 2015. Works in both 2D and 3D.
// Reference: Halfspace intersection with Qhull by Brad Barber
//            http://www.geom.uiuc.edu/graphics/pix/Special_Topics/Computational_Geometry/half.html
//
// Algorithm steps:
// 1. Find a point within the input poly.
// 2. Center this point onto the origin.
// 3. Adjust the planes by a skin factor.
// 4. Compute the dual vert of each plane. Each plane becomes a vertex.
//    c2v dual(c2h plane) { return c2V(plane.n.x / plane.d, plane.n.y / plane.d) }
// 5. Compute the convex hull of the dual verts. This is called the dual.
// 6. Compute the dual of the dual, this will be the poly to return.
// 7. Translate the poly away from the origin by the center point from step 2.
// 8. Return the inflated poly.
c2Poly c2InflatePoly(c2Poly poly, float skin_factor)
{
	c2v average = poly.verts[0];
	for (int i = 1; i < poly.count; ++i) {
		average = c2Add(average, poly.verts[i]);
	}
	average = c2Div(average, (float)poly.count);

	for (int i = 0; i < poly.count; ++i) {
		poly.verts[i] = c2Sub(poly.verts[i], average);
	}

	c2Poly dual = c2Dual(poly, skin_factor);
	poly = c2Dual(dual, 0);

	for (int i = 0; i < poly.count; ++i) {
		poly.verts[i] = c2Add(poly.verts[i], average);
	}

	return poly;
}

void c2Inflate(void* shape, C2_TYPE type, float skin_factor)
{
	switch (type)
	{
	case C2_TYPE_CIRCLE:
	{
		c2Circle* circle = (c2Circle*)shape;
		circle->r += skin_factor;
	}	break;

	case C2_TYPE_AABB:
	{
		c2AABB* bb = (c2AABB*)shape;
		c2v factor = c2V(skin_factor, skin_factor);
		bb->min = c2Sub(bb->min, factor);
		bb->max = c2Add(bb->max, factor);
	}	break;

	case C2_TYPE_CAPSULE:
	{
		c2Capsule* capsule = (c2Capsule*)shape;
		capsule->r += skin_factor;
	}	break;

	case C2_TYPE_POLY:
	{
		c2Poly* poly = (c2Poly*)shape;
		*poly = c2InflatePoly(*poly, skin_factor);
	}	break;
	}
}

int c2CircletoCircle(c2Circle A, c2Circle B)
{
	c2v c = c2Sub(B.p, A.p);
	float d2 = c2Dot(c, c);
	float r2 = A.r + B.r;
	r2 = r2 * r2;
	return d2 < r2;
}

int c2CircletoAABB(c2Circle A, c2AABB B)
{
	c2v L = c2Clampv(A.p, B.min, B.max);
	c2v ab = c2Sub(A.p, L);
	float d2 = c2Dot(ab, ab);
	float r2 = A.r * A.r;
	return d2 < r2;
}

int c2AABBtoAABB(c2AABB A, c2AABB B)
{
	int d0 = B.max.x < A.min.x;
	int d1 = A.max.x < B.min.x;
	int d2 = B.max.y < A.min.y;
	int d3 = A.max.y < B.min.y;
	return !(d0 | d1 | d2 | d3);
}

int c2AABBtoPoint(c2AABB A, c2v B)
{
	int d0 = B.x < A.min.x;
	int d1 = B.y < A.min.y;
	int d2 = B.x > A.max.x;
	int d3 = B.y > A.max.y;
	return !(d0 | d1 | d2 | d3);
}

int c2CircleToPoint(c2Circle A, c2v B)
{
	c2v n = c2Sub(A.p, B);
	float d2 = c2Dot(n, n);
	return d2 < A.r * A.r;
}

// see: http://www.randygaul.net/2014/07/23/distance-point-to-line-segment/
int c2CircletoCapsule(c2Circle A, c2Capsule B)
{
	c2v n = c2Sub(B.b, B.a);
	c2v ap = c2Sub(A.p, B.a);
	float da = c2Dot(ap, n);
	float d2;

	if (da < 0) d2 = c2Dot(ap, ap);
	else
	{
		float db = c2Dot(c2Sub(A.p, B.b), n);
		if (db < 0)
		{
			c2v e = c2Sub(ap, c2Mulvs(n, (da / c2Dot(n, n))));
			d2 = c2Dot(e, e);
		}
		else
		{
			c2v bp = c2Sub(A.p, B.b);
			d2 = c2Dot(bp, bp);
		}
	}

	float r = A.r + B.r;
	return d2 < r * r;
}

static inline float c2SignedDistPointToPlane_OneDimensional(float p, float n, float d)
{
	return p * n - d * n;
}

static inline float c2RayToPlane_OneDimensional(float da, float db)
{
	if (da < 0) return 0; // Ray started behind plane.
	else if (da * db >= 0) return 1.0f; // Ray starts and ends on the same of the plane.
	else // Ray starts and ends on opposite sides of the plane (or directly on the plane).
	{
		float d = da - db;
		if (d != 0) return da / d;
		else return 0; // Special case for super tiny ray, or AABB.
	}
}

int c2RaytoAABB(c2Ray A, c2AABB B, c2Raycast* out)
{
	c2v p0 = A.p;
	c2v p1 = c2Impact(A, A.t);
	c2AABB a_box;
	a_box.min = c2Minv(p0, p1);
	a_box.max = c2Maxv(p0, p1);

	// Test B's axes.
	if (!c2AABBtoAABB(a_box, B)) return 0;

	// Test the ray's axes (along the segment's normal).
	c2v ab = c2Sub(p1, p0);
	c2v n = c2Skew(ab);
	c2v abs_n = c2Absv(n);
	c2v half_extents = c2Mulvs(c2Sub(B.max, B.min), 0.5f);
	c2v center_of_b_box = c2Mulvs(c2Add(B.min, B.max), 0.5f);
	float d = c2Abs(c2Dot(n, c2Sub(p0, center_of_b_box))) - c2Dot(abs_n, half_extents);
	if (d > 0) return 0;

	// Calculate intermediate values up-front.
	// This should play well with superscalar architecture.
	float da0 = c2SignedDistPointToPlane_OneDimensional(p0.x, -1.0f, B.min.x);
	float db0 = c2SignedDistPointToPlane_OneDimensional(p1.x, -1.0f, B.min.x);
	float da1 = c2SignedDistPointToPlane_OneDimensional(p0.x,  1.0f, B.max.x);
	float db1 = c2SignedDistPointToPlane_OneDimensional(p1.x,  1.0f, B.max.x);
	float da2 = c2SignedDistPointToPlane_OneDimensional(p0.y, -1.0f, B.min.y);
	float db2 = c2SignedDistPointToPlane_OneDimensional(p1.y, -1.0f, B.min.y);
	float da3 = c2SignedDistPointToPlane_OneDimensional(p0.y,  1.0f, B.max.y);
	float db3 = c2SignedDistPointToPlane_OneDimensional(p1.y,  1.0f, B.max.y);
	float t0 = c2RayToPlane_OneDimensional(da0, db0);
	float t1 = c2RayToPlane_OneDimensional(da1, db1);
	float t2 = c2RayToPlane_OneDimensional(da2, db2);
	float t3 = c2RayToPlane_OneDimensional(da3, db3);

	// Calculate hit predicate, no branching.
	int hit0 = t0 < 1.0f;
	int hit1 = t1 < 1.0f;
	int hit2 = t2 < 1.0f;
	int hit3 = t3 < 1.0f;
	int hit = hit0 | hit1 | hit2 | hit3;

	if (hit)
	{
		// Remap t's within 0-1 range, where >= 1 is treated as 0.
		t0 = (float)hit0 * t0;
		t1 = (float)hit1 * t1;
		t2 = (float)hit2 * t2;
		t3 = (float)hit3 * t3;

		// Sort output by finding largest t to deduce the normal.
		if (t0 >= t1 && t0 >= t2 && t0 >= t3)
		{
			out->t = t0 * A.t;
			out->n = c2V(-1, 0);
		}
		
		else if (t1 >= t0 && t1 >= t2 && t1 >= t3)
		{
			out->t = t1 * A.t;
			out->n = c2V(1, 0);
		}
		
		else if (t2 >= t0 && t2 >= t1 && t2 >= t3)
		{
			out->t = t2 * A.t;
			out->n = c2V(0, -1);
		}
		
		else
		{
			out->t = t3 * A.t;
			out->n = c2V(0, 1);
		}

		return 1;
	} else return 0; // This can still numerically happen.
}

int c2RaytoPoly(c2Ray A, const c2Poly* B, const c2x* bx_ptr, c2Raycast* out)
{
	c2x bx = bx_ptr ? *bx_ptr : c2xIdentity();
	c2v p = c2MulxvT(bx, A.p);
	c2v d = c2MulrvT(bx.r, A.d);
	float lo = 0;
	float hi = A.t;
	int index = ~0;

	// test ray to each plane, tracking lo/hi times of intersection
	for (int i = 0; i < B->count; ++i)
	{
		float num = c2Dot(B->norms[i], c2Sub(B->verts[i], p));
		float den = c2Dot(B->norms[i], d);
		if (den == 0 && num < 0) return 0;
		else
		{
			if (den < 0 && num < lo * den)
			{
				lo = num / den;
				index = i;
			}
			else if (den > 0 && num < hi * den) hi = num / den;
		}
		if (hi < lo) return 0;
	}

	if (index != ~0)
	{
		out->t = lo;
		out->n = c2Mulrv(bx.r, B->norms[index]);
		return 1;
	}

	return 0;
}

// clip a segment to a plane
static int c2Clip(c2v* seg, c2h h)
{
	c2v out[2];
	int sp = 0;
	float d0, d1;
	if ((d0 = c2Dist(h, seg[0])) < 0) out[sp++] = seg[0];
	if ((d1 = c2Dist(h, seg[1])) < 0) out[sp++] = seg[1];
	if (d0 == 0 && d1 == 0) {
		out[sp++] = seg[0];
		out[sp++] = seg[1];
	} else if (d0 * d1 <= 0) out[sp++] = c2Intersect(seg[0], seg[1], d0, d1);
	seg[0] = out[0]; seg[1] = out[1];
	return sp;
}

static int c2SidePlanes(c2v* seg, c2v ra, c2v rb, c2h* h)
{
	c2v in = c2Norm(c2Sub(rb, ra));
	c2h left = { c2Neg(in), c2Dot(c2Neg(in), ra) };
	c2h right = { in, c2Dot(in, rb) };
	if (c2Clip(seg, left) < 2) return 0;
	if (c2Clip(seg, right) < 2) return 0;
	if (h) {
		h->n = c2CCW90(in);
		h->d = c2Dot(c2CCW90(in), ra);
	}
	return 1;
}

// clip a segment to the "side planes" of another segment.
// side planes are planes orthogonal to a segment and attached to the
// endpoints of the segment
static int c2SidePlanesFromPoly(c2v* seg, c2x x, const c2Poly* p, int e, c2h* h)
{
	c2v ra = c2Mulxv(x, p->verts[e]);
	c2v rb = c2Mulxv(x, p->verts[e + 1 == p->count ? 0 : e + 1]);
	return c2SidePlanes(seg, ra, rb, h);
}

static void c2AntinormalFace(c2Capsule cap, const c2Poly* p, c2x x, int* face_out, c2v* n_out)
{
	float sep = -FLT_MAX;
	int index = ~0;
	c2v n = c2V(0, 0);
	for (int i = 0; i < p->count; ++i)
	{
		c2h h = c2Mulxh(x, c2PlaneAt(p, i));
		c2v n0 = c2Neg(h.n);
		c2v s = c2CapsuleSupport(cap, n0);
		float d = c2Dist(h, s);
		if (d > sep)
		{
			sep = d;
			index = i;
			n = n0;
		}
	}
	*face_out = index;
	*n_out = n;
}

static void c2Incident(c2v* incident, const c2Poly* ip, c2x ix, c2v rn_in_incident_space)
{
	int index = ~0;
	float min_dot = FLT_MAX;
	for (int i = 0; i < ip->count; ++i)
	{
		float dot = c2Dot(rn_in_incident_space, ip->norms[i]);
		if (dot < min_dot)
		{
			min_dot = dot;
			index = i;
		}
	}
	incident[0] = c2Mulxv(ix, ip->verts[index]);
	incident[1] = c2Mulxv(ix, ip->verts[index + 1 == ip->count ? 0 : index + 1]);
}

static float c2CheckFaces(const c2Poly* A, c2x ax, const c2Poly* B, c2x bx, int* face_index)
{
	c2x b_in_a = c2MulxxT(ax, bx);
	c2x a_in_b = c2MulxxT(bx, ax);
	float sep = -FLT_MAX;
	int index = ~0;

	for (int i = 0; i < A->count; ++i)
	{
		c2h h = c2PlaneAt(A, i);
		int idx = c2Support(B->verts, B->count, c2Mulrv(a_in_b.r, c2Neg(h.n)));
		c2v p = c2Mulxv(b_in_a, B->verts[idx]);
		float d = c2Dist(h, p);
		if (d > sep)
		{
			sep = d;
			index = i;
		}
	}

	*face_index = index;
	return sep;
}
