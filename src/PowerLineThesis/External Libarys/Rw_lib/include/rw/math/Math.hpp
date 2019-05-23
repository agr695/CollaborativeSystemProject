/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#ifndef RW_MATH_MATH_HPP
#define RW_MATH_MATH_HPP

/**
 * @file rw/math/Math.hpp
 */
#include <cmath>

#include "rw/math/RPY.hpp"
#include "rw/math/Quaternion.hpp"
#include "rw/math/Transform3D.hpp"
#include "rw/math/Constants.hpp"


namespace rw { namespace math {

    /** @addtogroup math */
    /*@{*/

    /**
       @brief Utility functions for the rw::math module.
    */
    class Math
    {
    public:
    	Math() = delete;
    	~Math() = delete;

        /**
         * @brief this function converts a EAA object to a Quaternion
         *
         * @param roll [in] rotation around z
         * @param pitch [in] rotation around y
         * @param yaw [in] rotation around x
         *
         * @return a Quaternion object that represents the converted EAA
         */
        template <class A>
        static Rotation3D<A> zyxToRotation3D(A roll, A pitch, A yaw)
        {
            return RPY<A>(roll, pitch, yaw).toRotation3D();
        }

        /**
         * \brief Constructs a 3x3 skew-symmetric matrix \f$ S\in so(3)\f$
         * \param s [in] the \f$ s_x \f$, \f$ s_y \f$ and \f$ s_z \f$ of the matrix
         * \return The 3x3 skew-symmetric matrix \f$S\f$
         *
         * \f$
         * S =
         * \left[
         * \begin{array}{ccc}
         *    0 & -s_z &  s_y\\
         *  s_z &    0 & -s_x\\
         * -s_y &  s_x &    0
         * \end{array}
         * \right]
         * \f$
         */
        template<class R>
        static inline boost::numeric::ublas::bounded_matrix<R, 3, 3> skew(const boost::numeric::ublas::bounded_vector<R, 3>& s)
        {
            boost::numeric::ublas::bounded_matrix<R, 3, 3> S;
            S(0,0) =   0.0; S(0,1) = -s[2]; S(0,2) =  s[1];
            S(1,0) =  s[2]; S(1,1) =   0.0; S(1,2) = -s[0];
            S(2,0) = -s[1]; S(2,1) =  s[0]; S(2,2) =   0.0;
            return S;
        }

        /**
         * \brief Constructs a 3x3 skew-symmetric matrix \f$ S\in so(3)\f$
         * \param s [in] the \f$ s_x \f$, \f$ s_y \f$ and \f$ s_z \f$ of the matrix
         * \return The 3x3 skew-symmetric matrix \f$S\f$
         *
         * \f$
         * S =
         * \left[
         * \begin{array}{ccc}
         *    0 & -s_z &  s_y\\
         *  s_z &    0 & -s_x\\
         * -s_y &  s_x &    0
         * \end{array}
         * \right]
         * \f$
         */
        template<class R>
		static inline Eigen::Matrix<R, 3, 3> skew(const Vector3D<R>& s)
        {
			Eigen::Matrix<R, 3, 3> S;
            S(0,0) =   0.0; S(0,1) = -s[2]; S(0,2) =  s[1];
            S(1,0) =  s[2]; S(1,1) =   0.0; S(1,2) = -s[0];
            S(2,0) = -s[1]; S(2,1) =  s[0]; S(2,2) =   0.0;
            return S;
        }


        /**
         * @brief clamp val to either min or max
         *
         * @param val [in] the value that is to be clamped
         * @param min [in] the minimum allowed value
         * @param max [in] the maximum allowed value
         * @return the clamped value of val
         */

        static inline double clamp(double val, double min, double max)
        {
            if (val < min)
                return min;
            if(val > max)
                return max;
            return val;
        }
      
		
		/**
         * @brief Clamps values of \b q with \b min and \b max
         *
         * @param q [in] Values to clamp
         * @param min [min] The minimum value
         * @param max [min] The maximum value
         * @return The clamped values
         */
        static rw::math::Vector3D<> clamp(
            const rw::math::Vector3D<>& q,
            const rw::math::Vector3D<>& min,
            const rw::math::Vector3D<>& max)
        {
            assert(q.size() == min.size());
            assert(q.size() == max.size());

            Vector3D<> qres;
            for (size_t i = 0; i<3; i++)
                qres(i) = clamp(q(i), min(i), max(i));

            return qres;
        }


        // Global random number generation.

        /**
         * @brief A random double in the range [0, 1[ using a uniform distribution.
         *
         * @note Uses boost::random
         */
        static double ran();

        /**
         * @brief Seeds the random number generator.
         *
         * @note Uses boost::random
         */
        static void seed(unsigned seed);

        /**
         * @brief Seeds the random number generator with current time of day
         *
         * @note Uses boost::random
         */
		static void seed();

        /**
         * @brief A random double in the range [from, to[ using a uniform distribution.
         *
         * @note Uses boost::random
         */
        static double ran(double from, double to);

        /**
         * @brief A random integer in the range [from, to[ using a uniform distribution.
         *
         * @note Uses boost::random
         */
        static int ranI(int from, int to);

        /**
         * @brief Returns a random sample around \b mean with standard deviation \b sigma using the normal distribution.
         *
         * @note Uses boost::random
		 * @warning The number sequence generated can vary in different Boost versions (there is a known change in Boost 1.56.0).
         *
         * @param mean [in] Means value
         * @param sigma [in] Standard deviation
         * @return Random sample
         */
        static double ranNormalDist(double mean, double sigma);


		/**
		 * @brief Returns a uniformly distributed random orientation.
		 *
		 * @return Random orientation represented as a Quaternion
		 */
		template<class T>
		static rw::math::Quaternion<T> ranQuaternion() {
                    double u1 = Math::ran();
                    double u2 = Math::ran();
                    double u3 = Math::ran();
                    Quaternion<T> q(static_cast<T>(std::sqrt(1-u1)*sin(2*Pi*u2)), static_cast<T>(std::sqrt(1-u1)*cos(2*Pi*u2)), static_cast<T>(std::sqrt(u1)*sin(2*Pi*u3)), static_cast<T>(std::sqrt(u1)*cos(2*Pi*u3)) );
                    return q;
		}

		/**
		 * @brief Returns a uniformly distributed random orientation.
		 *
		 * @return Random orientation represented as a Rotation3D
		 */
		template<class T>
		static rw::math::Rotation3D<T> ranRotation3D() {
                    return ranQuaternion<T>().toRotation3D();
		}

		/**
		 * @brief Returns random Transform3D based on ranDir (using the standard normal distribution) and ranRotation3D (using a uniform distribution).
		 *
                 * @param translationLength [in] 
		 * @return Random Transform3D
		 */
         /*        template<class T>
                 static rw::math::Transform3D<T> ranTransform3D(const double translationLength = 1) {
                     rw::math::Q dir = ranDir(3, translationLength);
                     rw::math::Vector3D<T> translation(static_cast<T>(dir(0)), static_cast<T>(dir(1)), static_cast<T>(dir(2)));
                     return rw::math::Transform3D<T>(translation, ranRotation3D<T>());
                 }*/
        
        /**
         * @brief Rounds off to nearest integer
         *
         * With some compilers \b round can be found in math.h. This however does not
         * appear to be ansi C/C++ standard
         *
         * @param d [in] number to round
         * @return d rounded to nearest integer.
         */
        static double round(double d) { return floor(d + 0.5); }

        /**
          @brief The square of \b d

          @param d [in] Number to square
          @return d * d
        */
        template <class T>
        static inline T sqr(const T& d) { return d * d; }

        /**
         * @brief Returns vector with the absolute values
         *
         * Given a vector \f$v=[v_1,v_2,\ldots,v_n]\f$ then Abs(v) is defined as
         * \f$Abs(v)=[abs(v_1),abs(v_i),\ldots,abs(v_n)] \f$
         *
         * @param v [in] the vector \f$v\f$
         * @return the vector \f$Abs(v)\f$
         */
        template<class T>
        static boost::numeric::ublas::vector<T> abs(const boost::numeric::ublas::vector<T>& v)
        {
            boost::numeric::ublas::vector<T> result(v.size());
            for (size_t i = 0; i < v.size(); i++)
                result[i] = std::fabs(v[i]);

            return result;
        }

        /**
         * @brief Returns the smallest element of v
         *
         * If the vector has zero length, the method returns 0
         *
         * @param v [in] the vector v
         * @return the smallest element
         */
        template<class T>
        static T min(const boost::numeric::ublas::vector<T>& v)
        {
            if (v.size() == 0)
                return 0;

            T minval = v(0);
            for (size_t i = 1; i<v.size(); i++)
                if (v(i)<minval)
                    minval = v(i);
            return minval;
        }

        /**
         * @brief Returns the largest element of v
         *
         * If the vector has zero length, the method returns 0
         *
         * @param v [in] the vector v
         * @return the largest element
         */
        template<class T>
        static T max(const boost::numeric::ublas::vector<T>& v)
        {
            if (v.size() == 0)
                return 0;

            T maxval = v(0);
            for (size_t i = 1; i<v.size(); i++)
                if (v(i)>maxval)
                    maxval = v(i);
            return maxval;
        }

        /**
         * @brief Returns vector with the elementwise smallest elements of \b a and \b b
         *
         * @param a [in] the vector \b a
         * @param b [in] the vector \b b
         * @return Q with smallest elements
         */
        template<class T>
        static T min(const T& a, const T& b)
        {
            assert(a.size() == b.size());

            T result(a.size());
            for (size_t i = 0; i<a.size(); i++)
                result[i] = std::min(a[i], b[i]);
            return result;
        }

        /**
         * @brief Returns vector with the elementwise largest elements of \b a and \b b
         *
         * @param a [in] the vector \b a
         * @param b [in] the vector \b b
         * @return Q with largest elements
         */
        template<class T>
        static T max(const T& a, const T& b)
        {
            assert(a.size() == b.size());

            T result(a.size());
            for (size_t i = 0; i<a.size(); i++)
                result(i) = std::max(a[i], b[i]);
            return result;
        }

        /**
         * @brief Returns vector with the absolute values
         *
         * Given a vector \f$v=[v_1,v_2,\ldots,v_n]\f$ then Abs(v) is defined as
         * \f$Abs(v)=[abs(v_1),abs(v_i),\ldots,abs(v_n)] \f$
         *
         * @param v [in] the vector \f$v\f$
         * @return the vector \f$Abs(v)\f$
         */
        template<class T>
        static Vector3D<T> abs(const Vector3D<T>& v)
        {
            Vector3D<T> result;
            for (size_t i = 0; i<3; i++)
                result[i] = std::fabs(v[i]);
            return result;
        }

        /**
         * @brief Returns the smallest element of v
         *
         * @param v [in] the vector v
         * @return the smallest element
         */
        template<class T>
        static T min(const Vector3D<T>& v)
        {
            T minval = v(0);
            for (size_t i = 1; i<3; i++)
                if (v(i)<minval)
                    minval = v(i);
            return minval;
        }

        /**
         * @brief Returns the largest element of v
         *
         * @param v [in] the vector v
         * @return the largest element
         */
        template<class T>
        static T max(const Vector3D<T>& v)
        {
            T maxval = v(0);
            for (size_t i = 1; i<3; i++)
                if (v(i)>maxval)
                    maxval = v(i);
            return maxval;
        }

        /**
         * @brief Returns vector with the elementwise smallest elements of \b a and \b b
         *
         * @param a [in] the vector \b a
         * @param b [in] the vector \b b
         * @return Vector with smallest elements
         */
        template<class T>
        static Vector3D<T> min(const Vector3D<T>& a, const Vector3D<T>& b)
        {
            Vector3D<T> result;
            for (size_t i = 0; i<3; i++)
                result(i) = std::min(a[i], b[i]);
            return result;
        }

        /**
         * @brief Returns vector with the elementwise largest elements of \b a and \b b
         *
         * @param a [in] the vector \b a
         * @param b [in] the vector \b b
         * @return Vector with largest elements
         */
        template<class T>
        static Vector3D<T> max(const Vector3D<T>& a, const Vector3D<T>& b)
        {
            Vector3D<T> result;
            for (size_t i = 0; i<3; i++)
                result(i) = std::max(a[i], b[i]);
            return result;
        }

        /**
         * @brief Returns the sign of s
         *
         * If s < 0 it return 0. If s >= 0 then 1 is returned.
         *
         * @param s [in] The value for which to return the sign
         * @return The sign
         */
        static double sign(double s) { return s >= 0 ? 1 : -1; }

        /**
           @brief Exact implementation of ceil(log_2(n)) for n > 0.
        */
        static int ceilLog2(int n);
        
        /**
         * @brief Factorial 
         * The method does not implement any safe guards for negative numbers of overflow of numbers. 
         */
        static long long factorial(long long n) {
			return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
		}

        /**
         * @brief convert a math vector type to an vector of doubles. The input should
         * have the index operator () in order to use this conversion
         * @param tmp [in] input
         * @param size [in] length of tmp
         * @return vector of doubles
         */
        template <class ARR>
        static std::vector<double> toStdVector(const ARR& tmp, int size){
            std::vector<double> qvec(size);
            for(int i=0;i<size;i++){
                qvec[i] = tmp(i);
            }
            return qvec;
        }

        /**
         * @brief convert a math matrix type to an vector of doubles. The input should
         * have the index operator (x,y) in order to use this conversion
         * @param tmp [in] input matrix type
         * @param size1 [in] width of tmp
         * @param size2 [in] height of tmp
         * @return vector of doubles
         */
        template <class MAT>
        static std::vector<double> toStdVector(const MAT& tmp, int size1, int size2){
            std::vector<double> qvec(size1*size2);
            for(int i=0;i<size1;i++){
                for(int j=0;j<size2;j++){
                    qvec[i*size2+j] = tmp(i,j);
                }
            }
            return qvec;
        }

        /**
         * convert a vector of doubles to a vector math type. The math type should implement
         * the operator () in order to use this function.
         * @param data [in] the input
         * @param tmp [out] the output
         * @return reference to tmp
         */
        template <class T, class ARR>
        static ARR fromStdVector(const std::vector<T>& data, ARR& tmp){
            for(size_t i=0;i<data.size();i++){
                tmp(i) = data[i];
            }
            return tmp;
        }

        /**
         * convert a vector of doubles to a matrix math type. The math type should implement
         * the operator (i,j) in order to use this function.
         * @param data [in] the input
         * @param tmp [out] the output
         * @param size1 [in] the size of the first dimension of the matrix.
         * @param size2 [in] the size of the second dimension of the matrix.
         * @return reference to tmp
         */
        template <class T, class MAT>
        static MAT fromStdVectorToMat(const std::vector<T>& data, MAT& tmp, int size1, int size2){
            for(size_t i=0;(int)i<size1;i++){
                for(size_t j=0;(int)j<size2;j++){
                    tmp(i,j) = data[i*size2+j];
                }
            }
            return tmp;
        }

		/**
		 * @brief Implements an isNaN function
		 *
		 * Use to make sure code is independent of specific compile specific implementations
		 */
		static bool isNaN(double d);

		/**
		 * @brief Get a value for NaN.
		 *
		 * Use to make sure code is independent of specific compile specific implementations
		 *
		 * @return a double representation of NaN.
		 */
		static double NaN();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard