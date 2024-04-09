
#include "../geometry/spline.h"

template <typename T>
T Spline<T>::at(float time) const
{

	// A4T1b: Evaluate a Catumull-Rom spline

	// Given a time, find the nearest positions & tangent values
	// defined by the control point map.

	// Transform them for use with cubic_unit_spline

	// Be wary of edge cases! What if time is before the first knot,
	// before the second knot, etc...
	if (knots.empty())
		return T();
	if (knots.size() == 1)
	{
		auto it = knots.begin();
		return it->second;
	}

	std::map<float, T>::const_iterator first = knots.begin();
	auto last = --knots.end();
	if (time <= first->first)
		return first->second;
	if (time >= last->first)
		return last->second;

	auto pre_t = knots.begin(), post_t = knots.begin();
	T p0, p1, p2, p3;
	float t0, t1, t2, t3;
	post_t = knots.upper_bound(time);
	pre_t = --post_t;
	++post_t;
	p1 = pre_t->second, t1 = pre_t->first;
	p2 = post_t->second, t2 = post_t->first;

	if (post_t == last)
	{
		p3 = 2 * p2 - p1;
		t3 = 2 * t2 - t1;
	}
	else
	{
		++post_t;
		p3 = post_t->second;
		t3 = post_t->first;
		--post_t;
	}
	if (pre_t == first)
	{
		p0 = 2 * p1 - p2;
		t0 = 2 * t1 - t2;
	}
	else
	{
		--pre_t;
		p0 = pre_t->second;
		t0 = pre_t->first;
		++pre_t;
	}

	// return cubic_unit_spline(0.f, T(), T(), T(), T());
	T m0, m1;
	m0 = (p2 - p0) / (t2 - t0);
	m1 = (p3 - p1) / (t3 - t1);

	// normalization
	float t = (time - t1) / (t2 - t1);
	m0 *= (t2 - t1);
	m1 *= (t2 - t1);

	return cubic_unit_spline(t, p1, p2, m0, m1);
}

template <typename T>
T Spline<T>::cubic_unit_spline(float time, const T &position0, const T &position1,
							   const T &tangent0, const T &tangent1)
{

	// A4T1a: Hermite Curve over the unit interval

	// Given time in [0,1] compute the cubic spline coefficients and use them to compute
	// the interpolated value at time 'time' based on the positions & tangents

	// Note that Spline is parameterized on type T, which allows us to create splines over
	// any type that supports the * and + operators.
	if (time <= 0)
		return position0;
	if (time >= 1)
		return position1;

	float h00, h10, h01, h11;
	h00 = 2 * powf(time, 3.f) - 3 * time * time + 1;
	h10 = powf(time, 3.f) - 2 * time * time + time;
	h01 = -2 * powf(time, 3.f) + 3 * time * time;
	h11 = powf(time, 3.f) - time * time;
	auto ret = h00 * position0 + h10 * tangent0 + h01 * position1 + h11 * tangent1;

	return ret;
}

template class Spline<float>;
template class Spline<double>;
template class Spline<Vec4>;
template class Spline<Vec3>;
template class Spline<Vec2>;
template class Spline<Mat4>;
template class Spline<Spectrum>;
