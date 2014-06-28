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


#ifndef NODE_H
#define NODE_H

#include "initdata.h"
#include "config.h"
namespace vr{
    /*
     * Node
     */

    class Node : public eq::Node
    {
    private:
        InitData init_data;

    public:
        Node(eq::Config *parent);
        const InitData &getInitData() const;

    protected:
        virtual bool configInit(const eq::uint128_t &init_id);
        virtual bool configExit();
    };
}

#endif // NODE_H
