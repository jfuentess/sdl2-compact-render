/*
 * Created by Fernando Silva on 10/09/20.
 *
 * Copyright (C) 2018-current-year, Fernando Silva, all rights reserved.
 *
 *
 * Author's contact: Fernando Silva  <fernando.silva@udc.es>
 * Databases Lab, University of A Coruña. Campus de Elviña s/n. Spain
 *
 * DESCRIPTION
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifndef K3_TREE_SDSL_LIDAR_POINT_H
#define K3_TREE_SDSL_LIDAR_POINT_H

//#include <LAStools/LASlib/inc/lasreader.hpp>

//! Namespace for points library
namespace util_points {

    //*******************************************************//
    //******************* Point Base ************************//
    //*******************************************************//

    template<typename point_size_type=size_t>
    class point_base {

    public:
        point_size_type X{};
        point_size_type Y{};
        point_size_type Z{};

        //*******************//
        //*** CONSTRUCTOR ***//
        //*******************//
        point_base() = default;

        point_base (point_size_type pos_x, point_size_type pos_y, point_size_type pos_z) {
            this->X = pos_x;
            this->Y = pos_y;
            this->Z = pos_z;
        }

        //*******************//
        //**** BASIC OP. ****//
        //*******************//
        bool operator<(const point_base& p) const
        {
            if (this->X != p.X) return this->X < p.X;
            if (this->Y != p.Y) return this->Y < p.Y;
            if (this->Z != p.Z) return this->Z < p.Z;
            return false;
        }

        bool operator==(const point_base& p) const
        {
            return (this->X == p.X && this->Y == p.Y && this->Z == p.Z);
        }

        bool compare(const point_base &p) const
        {
            if (this->X != p.X) return this->X < p.X;
            if (this->X != p.Y) return this->Y < p.Y;
            if (this->Z != p.Z) return this->Z < p.Z;
            return false;
        }

        //*******************//
        //****** PRINT ******//
        //*******************//
        virtual void print() const {
            std::cout << "Point (" << this->X << ", " << this->Y << ", " << this->Z << ")" << std::endl;
        }
    };

}; // END NAMESPACE k3-tree

#endif //K3_TREE_SDSL_LIDAR_POINT_H
