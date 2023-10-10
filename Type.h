#ifndef __TYPEINFO_h
#define __TYPEINFO_h

template<typename _Tp, _Tp __v>
struct integral_constant
{
  static constexpr _Tp                  value = __v;
  typedef _Tp                           value_type;
  typedef integral_constant<_Tp, __v>   type;
  constexpr operator value_type() const noexcept { return value; }
};

using true_type = integral_constant<bool, true>;
using false_type = integral_constant<bool, false>;

template <typename T, typename U>
struct is_same : false_type {} ;
template <typename T>
struct is_same <T, T> : true_type {};

template <typename T>
struct is_callable {
    template <typename U, decltype(&U::operator()) = &U::operator()>
    struct checker {};
    template <typename U> static true_type  test(checker<U> *);
    static constexpr bool value = decltype(test<T>(nullptr))::value;
};

#endif;