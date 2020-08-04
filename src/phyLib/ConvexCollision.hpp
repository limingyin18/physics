#pragma once

#include <array>
#include <vector>
#include <queue>
#include <utility>
#include <algorithm>
#include <optional>

#include "PolyhedralConvexShape.hpp"


/** 
 * collision detection
 * @param shape1 object a
 * @param shape2 object b
 * 
 * @return witness points
 */
std::optional<Eigen::Vector3f>
collisionDetection(const PolyhedralConvexShape &shape1, 
    const PolyhedralConvexShape &shape2);

/**
 * EPA algorithm
 * @param shape1 object a
 * @param shape2 object b
 * @param simplex simplex return from GJK algorithm
 * 
 * @return penatration depth
 */
std::optional<Eigen::Vector3f>
EPAAlgorithm(const PolyhedralConvexShape &shape1, const PolyhedralConvexShape &shape2, 
    std::vector<Eigen::Vector3f> &simplex);

/**
 * GJK algorithm
 * @param shape1 object a
 * @param shape2 object b
 * 
 * @return simplex
 */
std::optional<std::vector<Eigen::Vector3f>>
GJKAlgorithm(const PolyhedralConvexShape &shape1, const PolyhedralConvexShape &shape2);

/**
 * Minkowski difference
 * 
 * @param shape1 object a
 * @param shape2 object b
 * @param dir search direction
 * 
 * @return support point
 */
Eigen::Vector3f
MinkowskiDifferenceSupport(const PolyhedralConvexShape &shape1, 
    const PolyhedralConvexShape &shape2, const Eigen::Vector3f &dir);

/**
 * point project to plane given by point and normal
 *
 * @param p point outside the plane
 * @param p1 point on the plane
 * @param n normal to the plane
 * 
 * @return q the projection point
 */
Eigen::Vector3f
pointToPlane(const Eigen::Vector3f &p, const Eigen::Vector3f &p1, 
    const Eigen::Vector3f &n);

/** 
 * point project to plane given by three points
 *
 * @param p point
 * @param p1 
 * @param p2
 * @param p3 three points define the plane
 * @return q the projection point
 */
Eigen::Vector3f pointToPlane(const Eigen::Vector3f &p,
    const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3);


/** 
 * Determine whether point P in triangle ABC
 * @param A
 * @param B
 * @param C three points define the triangle
 * @param P point testing
 * 
 * @return whether point P in triangle
 */
bool pointInTriangle(const Eigen::Vector3f &A, const Eigen::Vector3f &B,
    const Eigen::Vector3f &C, const Eigen::Vector3f &P);

/**
 * Determine whether origin in tetrahedron
 * @param s tetrahedron
 * 
 * @return whether origin in tetrahedron
 */
bool originInTetrahedron(std::vector<Eigen::Vector3f> &s);