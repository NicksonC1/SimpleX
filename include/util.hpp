#ifndef UTIL_HPP
#define UTIL_HPP

namespace genesis {
template <typename T>
int sgn(T value) {
	return (T(0) < value) - (value < T(0));
}
} // namespace genesis

#endif
