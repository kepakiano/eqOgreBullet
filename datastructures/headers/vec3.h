/*
* Copyright (C) 2014
* Sebastian Schmitz <sschmitz@informatik.uni-siegen.de>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef VEC3_H
#define VEC3_H

#include <cmath>

class Vec3
{
public:
    Vec3()
        : _x(0.0f), _y(0.0f), _z(0.0f)
    { }
    Vec3(float x, float y, float z)
        : _x(x), _y(y), _z(z)
    { }

    Vec3(float x)
        : _x(x), _y(x), _z(x)
    { }

    bool operator==(Vec3 rhs)
    {
        return _x == rhs._x && _y == rhs._y && _z == rhs._z;
    }

    bool operator!=(Vec3 rhs)
    {
        return !(*this == rhs);
    }

    const Vec3 operator +(const Vec3& rhs) const
    {
        return Vec3(_x + rhs.x(),
                    _y + rhs.y(),
                    _z + rhs.z()
                    );
    }

    const Vec3 operator -(const Vec3& rhs) const
    {
        return Vec3(_x - rhs.x(),
                    _y - rhs.y(),
                    _z - rhs.z()
                    );
    }

    const Vec3 operator *(const float rhs) const
    {
        return Vec3(_x * rhs,
                    _y * rhs,
                    _z * rhs
                    );
    }

    double getLength() const
    {
        return sqrt(_x * _x + _y * _y + _z * _z);
    }
    const Vec3 normalizedCopy() const
    {
        double length = getLength();
        return Vec3(_x / length, _y / length, _z / length);
    }

    void normalize()
    {
        double length = getLength();
        _x /= length;
        _y /= length;
        _z /= length;
    }

    Vec3 operator-() const
    {
        return Vec3(-_x, -_y, -_z);
    }

    float x() const {return _x;}
    float y() const {return _y;}
    float z() const {return _z;}

    void x(float curr_x) {_x = curr_x;}
    void y(float curr_y) {_y = curr_y;}
    void z(float curr_z) {_z = curr_z;}

    Vec3& operator=(const Vec3 & rhs)
    {
        if(this == &rhs) return *this;
        _x = rhs._x;
        _y = rhs._y;
        _z = rhs._z;
        return *this;
    }


private:
    float _x, _y, _z;
};

#endif // VEC3_H
