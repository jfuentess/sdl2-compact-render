/*  
 * Created by Fernando Silva on 13/06/18.
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

#ifndef K3_TREE_SDSL_K3_TREE_BASE
#define K3_TREE_SDSL_K3_TREE_BASE

#include <sdsl/vectors.hpp>
#include <util/utils_point.hpp>

//! Namespace for the k3-tree library
namespace k3tree {

    static const ushort K3_TREE_TYPE_BASIC = 0;
    static const ushort K3_TREE_TYPE_P3 = 1;
    static const ushort K3_TREE_TYPE_P1 = 1;
    static const ushort K3_TREE_TYPE_LP3 = 10;
    static const ushort K3_TREE_TYPE_LP1 = 11;
    static const ushort K3_TREE_TYPE_LP3_I = 100;
    static const ushort K3_TREE_TYPE_LP1_I = 110;

template<typename t_bv=sdsl::bit_vector,
        typename t_rank=typename t_bv::rank_1_type,
        typename t_point=util_points::point_base<>>
class k3_tree_base
{
    public:
        typedef sdsl::int_vector<>::size_type                   size_type;
        typedef sdsl::int_vector<>::size_type                   pos_type;
        typedef t_point                                         point_type;

    protected:

        // Basic params
        uint8_t     k_k1{};
        uint8_t     k_k2{};
        uint16_t    k_level_k1{};
        uint16_t    k_height{};
        pos_type    k_size{};
        pos_type    k_real_size_x{};
        pos_type    k_real_size_y{};
        pos_type    k_real_size_z{};

        //! Bit array to store all the bits of the tree, except those in the
        //! last level.
        t_bv        k_t;
        t_rank      k_t_rank;

        //! Bit array to store a bit for each 0 in k_t (1 -> it as at least one point | 0 -> empty node)
        t_bv        k_l;

        // Helpers
        uint        k_k1_3{};

    public:

        const size_type &size = k_size;

    public:
        //*******************************************************//
        //******************* CONSTRUCTOR ***********************//
        //*******************************************************//
        k3_tree_base() = default;;

        k3_tree_base(const k3_tree_base& tr)
        {
            *this = tr;
        }

        k3_tree_base(k3_tree_base&& tr)
        {
            *this = std::move(tr);
        }

        k3_tree_base(pos_type size_x, pos_type size_y, pos_type size_z, uint8_t k1, uint8_t k2, uint16_t levels_k1) {
            init(size_x, size_y, size_z, k1, k2, levels_k1);
        }

        //*******************************************************//
        //*************** BASIC OPERATIONS **********************//
        //*******************************************************//

        //! Move assignment operator
        k3_tree_base& operator=(k3_tree_base&& tr)
        {
            if (this != &tr) {
                // Basic params
                k_k1 = std::move(tr.k_k1);
                k_k2 = std::move(tr.k_k2);
                k_height = std::move(tr.k_height);
                k_size = std::move(tr.k_size);

                // Bitmaps
                k_t = std::move(tr.k_t);
                k_t_rank = std::move(tr.k_t_rank);
                k_t_rank.set_vector(&k_t);

                k_l = std::move(tr.k_l);
            }
            return *this;
        }

        //! Assignment operator
        k3_tree_base& operator=(const k3_tree_base& tr)
        {
            if (this != &tr) {
                // Basic params
                k_k1 = tr.k_k1;
                k_k2 = tr.k_k2;
                k_height = tr.k_height;
                k_size = tr.k_size;

                // Bitmaps
                k_t = tr.k_t;
                k_t_rank = tr.k_t_rank;
                k_t_rank.set_vector(&k_t);

                k_l = tr.k_l;
            }
            return *this;
        }

        //! Swap operator
        void swap(k3_tree_base& tr)
        {
            if (this != &tr) {
                // Basic params
                std::swap(k_k1, tr.k_k1);
                std::swap(k_k2, tr.k_k2);
                std::swap(k_height, tr.k_height);
                std::swap(k_size, tr.k_size);

                // Bitmaps
                std::swap(k_t, tr.k_t);
                sdsl::util::swap_support(k_t_rank, tr.k_t_rank, &k_t, &(tr.k_t));

                std::swap(k_l, tr.k_l);
            }
        }

        //! Equal operator
        bool operator==(const k3_tree_base& tr) const
        {
            if (k_k1 != tr.k_k1 || k_k2 != tr.k_k2 || k_height != tr.k_height || k_size != tr.k_size)
                return false;
            if (k_t.size() != tr.k_t.size() || k_l.size() != tr.k_l.size())
                return false;
            for (unsigned i = 0; i < k_t.size(); i++)
                if (k_t[i] != tr.k_t[i])
                    return false;
            for (unsigned i = 0; i < k_l.size(); i++)
                if (k_l[i] != tr.k_l[i])
                    return false;
            return true;
        }

        //*******************************************************//
        //********************** GETTERS ************************//
        //*******************************************************//
        virtual pos_type get_min_size_x() const {return 0;}
        virtual pos_type get_min_size_y() const {return 0;}
        virtual pos_type get_min_size_z() const {return 0;}
        virtual pos_type get_max_size_x() const {return k_real_size_x;}
        virtual pos_type get_max_size_y() const {return k_real_size_y;}
        virtual pos_type get_max_size_z() const {return k_real_size_z;}

        //*******************************************************//
        //********************** QUERIES ************************//
        //*******************************************************//

        virtual bool get(pos_type pos_x, pos_type pos_y, pos_type pos_z) const=0;

        virtual size_type get_region(pos_type i_pos_x, pos_type i_pos_y, pos_type i_pos_z,
                                     pos_type e_pos_x, pos_type e_pos_y, pos_type e_pos_z,
                                     std::vector<point_type> &result) const = 0;

        virtual size_type get_region(pos_type i_pos_x, pos_type i_pos_y,
                                 pos_type e_pos_x, pos_type e_pos_y,
                                 std::vector<point_type> &result) const {
            return get_region(i_pos_x, i_pos_y, get_min_size_x(), e_pos_x, e_pos_y, get_max_size_z(), result);
        };

        //*******************************************************//
        //*********************** FILE **************************//
        //*******************************************************//

        virtual size_type serialize(std::ostream& out, sdsl::structure_tree_node* v, std::string name) const = 0;
        virtual void load(std::istream& in) = 0;

        //*******************************************************//
        //*********************** INFO **************************//
        //*******************************************************//

        virtual void print() const = 0;

        //*******************************************************//
        //*********************** TEST **************************//
        //*******************************************************//

        virtual bool check_values(const std::string& data_file_name){
            std::ifstream values_file(data_file_name);
            assert(values_file.is_open() && values_file.good());

            bool res = check_values(values_file);
            values_file.close();
            return res;
        }

        virtual bool check_values(std::istream& data_file) {
            // Read file
            size_type pos_x, pos_y, pos_z;
            while (!data_file.eof() && data_file.good()) {

                // Get position (x, y, z)
                sdsl::read_member(pos_x, data_file);
                sdsl::read_member(pos_y, data_file);
                sdsl::read_member(pos_z, data_file);

                if (!get(pos_x, pos_y, pos_z)) {
    #ifndef NDEBUG
                    std::cout << "Failed point (" << pos_x << ", " << pos_y << ", " << pos_z << ") " << std::endl;
    #endif
                    return false;
                }

            }
            return true;
        }

        virtual bool check_values(const std::vector<point_type>& points) {

            // Read file
            for (auto point : points) {

                if (!get(point.X, point.Y, point.Z)) {
    #ifndef NDEBUG
                    std::cout << "Failed point (" << point.X << ", " << point.Y << ", " << point.Z << ") " << std::endl;
    #endif
                    return false;
                }
            }
            return true;
        }

protected:

    //*******************************************************//
    //****************** HELPER INIT ************************//
    //*******************************************************//

    void init(pos_type size_x, pos_type size_y, pos_type size_z, uint8_t k1, uint8_t k2, uint16_t levels_k1) {

        assert(size_x > 0);
        assert(size_y > 0);
        assert(size_z > 0);

        // Set params
        k_k1 = k1;
        k_k2 = k2;
        k_level_k1 = levels_k1;
        k_k1_3 = pow(k_k1, 3);

        // Store real size
        k_real_size_x = size_x;
        k_real_size_y = size_y;
        k_real_size_z = size_z;

        size_type size_max = std::max(std::max(size_x, size_y), size_z);

        // TODO check this for k2
        k_height = std::ceil(std::log(size_max)/std::log(k_k1));
        k_height = k_height > 1 ? k_height : 1; // If size == 0

        // Virtual size of the cube
        k_size = pow(k_k1, k_height);
    }

    size_type init_from_file(const std::string& filename, uint8_t k1, uint8_t k2, uint8_t levels_k1,
                             std::vector<point_type> &points) {

        // Open data file
        std::ifstream data_file(filename);
        assert(data_file.is_open() && data_file.good());

        // Read file and create conceptual tree
        pos_type pos_x, pos_y, pos_z;
        pos_type size_x = 0, size_y = 0, size_z = 0;

        while (!data_file.eof() && data_file.good()) {

            // Get position (x, y, z)
            sdsl::read_member(pos_x, data_file);
            sdsl::read_member(pos_y, data_file);
            sdsl::read_member(pos_z, data_file);

            // Store point into vector
            points.push_back({pos_x, pos_y, pos_z});

            // Calculate max size
            size_x = std::max(size_x, pos_x);
            size_y = std::max(size_y, pos_y);
            size_z = std::max(size_z, pos_z);
        }

        // Remove duplicates
        std::sort(points.begin(), points.end());
        points.erase(std::unique(points.begin(), points.end()), points.end());

        // Init attributes
        init(size_x + 1, size_y + 1, size_z + 1, k1, k2, levels_k1);

        return points.size();
    }

    //*******************************************************//
    //****************** HELPER FILE ************************//
    //*******************************************************//

    virtual size_type serialize_base(std::ostream& out, ushort k3_tree_type, sdsl::structure_tree_node* v,
                                     std::string &name) const {
        sdsl::structure_tree_node* child = sdsl::structure_tree::add_child(
                v, name, sdsl::util::class_name(*this));
        size_type written_bytes = 0;

        written_bytes += write_member(k3_tree_type, out, child, "k3_tree_type");
        written_bytes += k_t.serialize(out, child, "t");
        written_bytes += k_t_rank.serialize(out, child, "t_rank");
        written_bytes += k_l.serialize(out, child, "l");
        written_bytes += write_member(k_k1, out, child, "k1");
        written_bytes += write_member(k_k2, out, child, "k2");
        written_bytes += write_member(k_level_k1, out, child, "level_k1");
        written_bytes += write_member(k_height, out, child, "height");
        written_bytes += write_member(k_size, out, child, "size");


        written_bytes += write_member(k_real_size_x, out, child, "real_size_x");
        written_bytes += write_member(k_real_size_y, out, child, "real_size_y");
        written_bytes += write_member(k_real_size_z, out, child, "real_size_z");

        sdsl::structure_tree::add_size(child, written_bytes);
        return written_bytes;
    };

    void load_base(std::istream& in) {
        ushort type;
        sdsl::read_member(type, in);
        k_t.load(in);
        k_t_rank.load(in);
        k_t_rank.set_vector(&k_t);
        k_l.load(in);
        sdsl::read_member(k_k1, in);
        sdsl::read_member(k_k2, in);
        sdsl::read_member(k_level_k1, in);
        sdsl::read_member(k_height, in);
        sdsl::read_member(k_size, in);

        sdsl::read_member(k_real_size_x, in);
        sdsl::read_member(k_real_size_y, in);
        sdsl::read_member(k_real_size_z, in);

        k_k1_3 = pow(k_k1, 3);

    };

}; // END CLASS k3_tree_base

} // END NAMESPACE sdsl

#endif // K3_TREE_SDSL_K3_TREE_BASE
