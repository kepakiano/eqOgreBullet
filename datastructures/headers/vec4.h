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


#ifndef VEC4_H
#define VEC4_H

class Vec4
{
public:
    Vec4()
        : _x(0.0f), _y(0.0f), _z(0.0f), _w(0.0f)
    { }
    Vec4(float x, float y, float z, float w)
        : _x(x), _y(y), _z(z), _w(w)
    { }

    bool operator==(Vec4 rhs){
        return _x == rhs._x && _y == rhs._y && _z == rhs._z && _w == rhs._w;
    }

    bool operator!=(Vec4 rhs)
    {
        return !(*this == rhs);
    }

    float x() const {return _x;}
    float y() const {return _y;}
    float z() const {return _z;}
    float w() const {return _w;}

    void x(float x) {_x = x;}
    void y(float y) {_y = y;}
    void z(float z) {_z = z;}
    void w(float w) {_w = w;}

    Vec4& operator=(const Vec4 & rhs)
    {
        if(this == &rhs) return *this;
        _x = rhs._x;
        _y = rhs._y;
        _z = rhs._z;
        _w = rhs._w;
        return *this;
    }

private:
    float _x, _y, _z, _w;
};

#endif // VEC4_H
