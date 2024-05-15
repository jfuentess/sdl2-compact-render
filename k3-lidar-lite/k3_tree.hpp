/*  
 * Created by Fernando Silva on 3/04/18.
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

#ifndef K3_TREE_SDSL_K3_TREE
#define K3_TREE_SDSL_K3_TREE

#include <cstdint>
#include <sdsl/vectors.hpp>
#include <queue>
#include <tuple>

#include <k3_tree_base.hpp>

#include <glm/glm.hpp>

//! Namespace for the k3-tree library
namespace k3tree {


template<typename t_bv=sdsl::bit_vector,
        typename t_rank=typename t_bv::rank_1_type>
class k3_tree : public k3_tree_base<t_bv, t_rank>
{
    public:

        typedef k3_tree_base<t_bv, t_rank>      k3_tree_p;
        typedef t_bv                            bit_vector_type;

        typedef k3_tree_base<>::size_type       size_type;
        typedef k3_tree_base<>::pos_type        pos_type;
        typedef k3_tree_base<>::point_type      point_type;

        // (Initial position, end position, submatrix size, base x coordinate, base y coordinate, base z coordinate)
        typedef std::tuple<pos_type, pos_type, pos_type, pos_type, pos_type, pos_type > t_part_tuple;

    private:

    public:

        //*******************************************************//
        //******************* CONSTRUCTOR ***********************//
        //*******************************************************//
        k3_tree() = default;

        k3_tree(const k3_tree& tr)
        {
            *this = tr;
        }

        k3_tree(k3_tree&& tr)
        {
            *this = std::move(tr);

        }

        k3_tree(std::string filename, uint8_t k1=2, uint8_t k2=2, uint8_t levels_k1=2)
        {
            // Init structure
            std::vector<point_type> points;
            k3_tree_base<t_bv, t_rank>::init_from_file(filename, k1, k2, levels_k1, points);

            // Build structures
            build(points);
        }

        k3_tree(std::vector<point_type> &points, pos_type size_x, pos_type size_y, pos_type size_z,
                uint8_t k1, uint8_t k2, uint8_t levels_k1) : k3_tree_base<t_bv, t_rank>(size_x, size_y, size_z, k1, k2, levels_k1)
        {
            // Build structures
            build(points);
        }

        //*******************************************************//
        //*************** BASIC OPERATIONS **********************//
        //*******************************************************//

        //! Move assignment operator
        k3_tree& operator=(k3_tree&& tr)
        {
            if (this != &tr) {
                k3_tree_p::operator=(tr);
            }
            return *this;
        }

        //! Assignment operator
        k3_tree& operator=(k3_tree& tr)
        {
            if (this != &tr) {
                k3_tree_p::operator=(tr);
            }
            return *this;
        }

        //! Swap operator
        void swap(k3_tree& tr)
        {
            if (this != &tr) {
                k3_tree_p::swap(tr);
            }
        }

        //! Equal operator
        bool operator==(const k3_tree& tr) const
        {
            return k3_tree_p::operator==(tr);
        }

        //*******************************************************//
        //********************** QUERIES ************************//
        //*******************************************************//

        //*********************//
        //***** GET POINT *****//
        //*********************//
        bool get(pos_type pos_x, pos_type pos_y, pos_type pos_z) const {
            assert(pos_x < this->k_size);
            assert(pos_y < this->k_size);
            assert(pos_z < this->k_size);

            if (this->k_t.size() == 0) {
                // Empty matrix
                return false;
            }

            if (pos_x > this->k_real_size_x || pos_y > this->k_real_size_y || pos_z > this->k_real_size_z) {
                return false;
            }

            // Size of the submatrix at the current level
            pos_type submatrix_size_l = this->k_size;
            size_type node_pos;
            size_type children_pos = 0;
            uint8_t k = this->k_k1;


            for (uint16_t l = 0; l < this->k_height-1; l++) {
                submatrix_size_l /= k;
                node_pos = (pos_x / submatrix_size_l) * k * k + pos_y / submatrix_size_l * k + pos_z / submatrix_size_l;
                node_pos += children_pos;

                if (this->k_t[node_pos] == 0) {
                    return false; // Empty submatrix
                } else {
                    // Go to next level
                    children_pos = this->k_t_rank(node_pos+1) * this->k_k1_3;
                }

                // Calculate local position on the current submatrix
                pos_x %= submatrix_size_l;
                pos_y %= submatrix_size_l;
                pos_z %= submatrix_size_l;
            }


            // Last level
            submatrix_size_l /= k;
            node_pos = (pos_x / submatrix_size_l) * k * k + pos_y / submatrix_size_l * k + pos_z / submatrix_size_l;
            node_pos += children_pos;
            return (this->k_l[node_pos - this->k_t.size()] == 1);
        }

        bool get_custom(pos_type pos_x, pos_type pos_y, pos_type pos_z, float real_pos_x, float real_pos_y, float real_pos_z, float &node_size, glm::vec3 &proportional_position) const {
            assert(pos_x < this->k_size);
            assert(pos_y < this->k_size);
            assert(pos_z < this->k_size);

            if (this->k_t.size() == 0) {
                // Empty matrix
                return false;
            }

            // THIS WAS COMMENTED BECAUSE IT RETURNED WITHOUT GIVING node_size AND proportional_position VARIABLES
            //if (pos_x > this->k_real_size_x || pos_y > this->k_real_size_y || pos_z > this->k_real_size_z) {
            //    return false;
            //}

            // Size of the submatrix at the current level
            pos_type submatrix_size_l = this->k_size;
            size_type node_pos;
            size_type children_pos = 0;
            uint8_t k = this->k_k1;

            node_size = submatrix_size_l;
            proportional_position.x = real_pos_x / (float) submatrix_size_l;
            proportional_position.y = real_pos_y / (float) submatrix_size_l;
            proportional_position.z = real_pos_z / (float) submatrix_size_l;

            for (uint16_t l = 0; l < this->k_height-1; l++) {
                submatrix_size_l /= k;
                node_size = submatrix_size_l;

                node_pos = (pos_x / submatrix_size_l) * k * k + pos_y / submatrix_size_l * k + pos_z / submatrix_size_l;
                node_pos += children_pos;

                //  JUST MOVED THIS PART TO k3_tree.hpp:245
                //if (this->k_t[node_pos] == 0) {
                //    return false; // Empty submatrix
                //} else {
                //    // Go to next level
                //    children_pos = this->k_t_rank(node_pos+1) * this->k_k1_3;
                //}

                // Calculate local position on the current submatrix
                pos_type before = pos_x;
                pos_x %= submatrix_size_l;
                real_pos_x -= (before - pos_x);

                before = pos_y;
                pos_y %= submatrix_size_l;
                real_pos_y -= (before - pos_y);

                before = pos_z;
                pos_z %= submatrix_size_l;
                real_pos_z -= (before - pos_z);

                node_size = submatrix_size_l;
                proportional_position.x = real_pos_x / (float) submatrix_size_l;
                proportional_position.y = real_pos_y / (float) submatrix_size_l;
                proportional_position.z = real_pos_z / (float) submatrix_size_l;

                if (this->k_t[node_pos] == 0) {
                    return false; // Empty submatrix
                } else {
                    // Go to next level
                    children_pos = this->k_t_rank(node_pos+1) * this->k_k1_3;
                }
            }

            // Last level
            submatrix_size_l /= k;
            node_pos = (pos_x / submatrix_size_l) * k * k + pos_y / submatrix_size_l * k + pos_z / submatrix_size_l;
            node_pos += children_pos;

            pos_type before = pos_x;
            pos_x %= submatrix_size_l;
            real_pos_x -= (before - pos_x);

            before = pos_y;
            pos_y %= submatrix_size_l;
            real_pos_y -= (before - pos_y);

            before = pos_z;
            pos_z %= submatrix_size_l;
            real_pos_z -= (before - pos_z);

            node_size = submatrix_size_l;
            proportional_position.x = real_pos_x / (float) submatrix_size_l;
            proportional_position.y = real_pos_y / (float) submatrix_size_l;
            proportional_position.z = real_pos_z / (float) submatrix_size_l;

	        if(node_pos - this->k_t.size() > this->k_l.size())
                return false;
            return (this->k_l[node_pos - this->k_t.size()] == 1);
        }

        //*********************//
        //***** GET REGION ****//
        //*********************//
        size_type get_region_2(pos_type i_pos_x, pos_type i_pos_y, pos_type i_pos_z,
                               pos_type e_pos_x, pos_type e_pos_y, pos_type e_pos_z,
                               std::vector<point_type> &result) {
            assert(i_pos_x < this->k_size && e_pos_x < this->k_size);
            assert(i_pos_y < this->k_size && e_pos_y < this->k_size);
            assert(i_pos_z < this->k_size && e_pos_z < this->k_size);
            assert(i_pos_x <= e_pos_x);
            assert(i_pos_y <= e_pos_y);
            assert(i_pos_z <= e_pos_z);

            if (i_pos_x > this->k_size || i_pos_y > this->k_size || i_pos_z > this->k_size) {
                return 0;
            }

            return get_region_r(i_pos_x, i_pos_y, i_pos_z,
                                std::min(e_pos_x, this->k_size), std::min(e_pos_y, this->k_size), std::min(e_pos_z, this->k_size), result,
                                0, 0, 0, this->k_size, 0, 0);
        }

        size_type get_region_2_custom(float pos_x, float pos_y, float pos_z,
                             std::vector<point_type> &result,
                             float &node_size, glm::vec3 &proportional_position) const {

            pos_type i_pos_x = pos_x;
            pos_type i_pos_y = pos_y;
            pos_type i_pos_z = pos_z;
            
            assert(i_pos_x < this->k_size);
            assert(i_pos_y < this->k_size);
            assert(i_pos_z < this->k_size);

            if (i_pos_x > this->k_size || i_pos_y > this->k_size || i_pos_z > this->k_size) {
                return 0;
            }

            return get_region_r_custom(i_pos_x, i_pos_y, i_pos_z,
                                std::min(i_pos_x, this->k_size), std::min(i_pos_y, this->k_size), std::min(i_pos_z, this->k_size), result,
                                0, 0, 0, this->k_size, 0, 0);
        }

        size_type get_region(pos_type i_pos_x, pos_type i_pos_y, pos_type i_pos_z,
                             pos_type e_pos_x, pos_type e_pos_y, pos_type e_pos_z,
                             std::vector<point_type> &result) const {

            if (i_pos_x > this->k_size || i_pos_y > this->k_size || i_pos_z > this->k_size) {
                return 0;
            }

            typedef std::tuple<pos_type, pos_type, pos_type, pos_type, pos_type, pos_type,
                    size_type, pos_type, pos_type, pos_type, size_type, uint16_t> t_part_tuple_r;
            std::stack<t_part_tuple_r> q;

            q.push(t_part_tuple_r(i_pos_x, i_pos_y, i_pos_z, e_pos_x, e_pos_y, e_pos_z,
                                this->k_size, 0, 0, 0, 0, 0));


            uint8_t k = this->k_k1;
            pos_type x_i_b, x_e_b, y_i_b, y_e_b, z_i_b, z_e_b;  // First and end child
            pos_type xi, xe, yi, ye, zi, ze;                    // Local positions
            pos_type bx, by, bz;                                // Bases positions
            size_type sub_size;                                 // Size of each submatrix
            size_type pos, pos_children;
            uint16_t l;

            while (!q.empty()) {
                std::tie(xi, yi, zi, xe, ye, ze, sub_size, bx, by, bz, pos_children, l) = q.top();
                q.pop();

                // Calculate submatrix size
                sub_size = sub_size / k;

                // For each child that has cells that overlap with the searched region
                x_i_b = xi/sub_size;
                x_e_b = xe/sub_size;
                y_i_b = yi/sub_size;
                y_e_b = ye/sub_size;
                z_i_b = zi/sub_size;
                z_e_b = ze/sub_size;


                for (size_type x = x_i_b; x <= x_e_b; x++) {
                    for (size_type y = y_i_b; y <= y_e_b; y++) {
                        for (size_type z = z_i_b; z <= z_e_b; z++) {
                            pos = pos_children + x * k * k + y * k + z; // Position of the current child

                            if (l < (this->k_height-1)) {
                                // Internal nodes
                                if (this->k_t[pos] == 1) {
                                    // Continue with the search process
                                    q.push(t_part_tuple_r(x == x_i_b ? xi % sub_size : 0,
                                                        y == y_i_b ? yi % sub_size : 0,
                                                        z == z_i_b ? zi % sub_size : 0,
                                                        x == x_e_b ? xe % sub_size : sub_size - 1,
                                                        y == y_e_b ? ye % sub_size : sub_size - 1,
                                                        z == z_e_b ? ze % sub_size : sub_size - 1,
                                                        sub_size, bx + x * sub_size, by + y * sub_size,  bz + z * sub_size,
                                                        this->k_t_rank(pos + 1) * this->k_k1_3, l+1));

                                } // ENF IF t[pos] == 1
                            } else {
                                // Leaves nodes
                                if (this->k_l[pos - this->k_t.size()] == 1) {
                                    result.push_back({bx + x, by + y, bz + z});
                                }
                            } // END IF check level
                        } // END FOR z
                    } // END FOR y
                } // END FOR x
            } // END WHILE queue
            return result.size();
        }

        size_type get_region_custom(float pos_x, float pos_y, float pos_z,
                             std::vector<point_type> &result,
                             float &node_size, glm::vec3 &proportional_position) const {

            pos_type i_pos_x = pos_x;
            pos_type i_pos_y = pos_y;
            pos_type i_pos_z = pos_z;

            if (i_pos_x > this->k_size || i_pos_y > this->k_size || i_pos_z > this->k_size) {
                return 0;
            }

            typedef std::tuple<pos_type, pos_type, pos_type, pos_type, pos_type, pos_type,
                    size_type, pos_type, pos_type, pos_type, size_type, uint16_t> t_part_tuple_r;
            std::stack<t_part_tuple_r> q;

            q.push(t_part_tuple_r(i_pos_x, i_pos_y, i_pos_z, i_pos_x, i_pos_y, i_pos_z,
                                this->k_size, 0, 0, 0, 0, 0));


            uint8_t k = this->k_k1;
            pos_type x_i_b, x_e_b, y_i_b, y_e_b, z_i_b, z_e_b;  // First and end child
            pos_type xi, xe, yi, ye, zi, ze;                    // Local positions
            pos_type bx, by, bz;                                // Bases positions
            size_type sub_size;                                 // Size of each submatrix
            size_type pos, pos_children;
            uint16_t l;

            while (!q.empty()) {
                std::tie(xi, yi, zi, xe, ye, ze, sub_size, bx, by, bz, pos_children, l) = q.top();
                q.pop();

                node_size = sub_size;
                proportional_position.x = pos_x / (float) sub_size;
                proportional_position.y = pos_y / (float) sub_size;
                proportional_position.z = pos_z / (float) sub_size;

                // Calculate submatrix size
                sub_size = sub_size / k;

                // For each child that has cells that overlap with the searched region
                x_i_b = xi/sub_size;
                x_e_b = xe/sub_size;
                y_i_b = yi/sub_size;
                y_e_b = ye/sub_size;
                z_i_b = zi/sub_size;
                z_e_b = ze/sub_size;


                for (size_type x = x_i_b; x <= x_e_b; x++) {
                    for (size_type y = y_i_b; y <= y_e_b; y++) {
                        for (size_type z = z_i_b; z <= z_e_b; z++) {
                            pos = pos_children + x * k * k + y * k + z; // Position of the current child

                            if (l < (this->k_height-1)) {
                                // Internal nodes
                                if (this->k_t[pos] == 1) {
                                    // Continue with the search process
                                    q.push(t_part_tuple_r(x == x_i_b ? xi % sub_size : 0,
                                                        y == y_i_b ? yi % sub_size : 0,
                                                        z == z_i_b ? zi % sub_size : 0,
                                                        x == x_e_b ? xe % sub_size : sub_size - 1,
                                                        y == y_e_b ? ye % sub_size : sub_size - 1,
                                                        z == z_e_b ? ze % sub_size : sub_size - 1,
                                                        sub_size, bx + x * sub_size, by + y * sub_size,  bz + z * sub_size,
                                                        this->k_t_rank(pos + 1) * this->k_k1_3, l+1));

                                } // ENF IF t[pos] == 1
                            } else {
                                // Leaves nodes
                                if (this->k_l[pos - this->k_t.size()] == 1) {
                                    result.push_back({bx + x, by + y, bz + z});
                                }
                            } // END IF check level
                        } // END FOR z
                    } // END FOR y
                } // END FOR x
            } // END WHILE queue
            return result.size();
        }


        //*******************************************************//
        //********************** FILE ***************************//
        //*******************************************************//

        //! Serialize to a stream
        /*! Serialize the k3_tree data structure
         *  \param out Outstream to write the k3_tree.
         *  \param v
         *  \param string_name
         *  \returns The number of written bytes.
         */
        size_type serialize(std::ostream& out, sdsl::structure_tree_node* v=nullptr,
                            std::string name="") const
        {
            return k3_tree_p::serialize_base(out, K3_TREE_TYPE_BASIC, v, name);
        }


        //! Load from istream
        /*! Serialize the k3_tree from the given istream.
         *  \param istream Stream to load the k3_tree from.
         */
        void load(std::istream& in)
        {
            k3_tree_p::load_base(in);
        }

    //*******************************************************//
    //*********************** INFO **************************//
    //*******************************************************//
        virtual void print() const {
            std::cout << "k3-tree with size " << this->k_size << " and height " << this->k_height << std::endl;
            std::cout << "k1 = " << (uint)this->k_k1 << " | k2 = " << (uint)this->k_k2 << std::endl;
            std::cout << "Bitmap t (size " << this->k_t.size() << ") : ";
    //        for (size_type p = 0; p < k_t.size(); p++) {
    //            std::cout << k_t[p];
    //        }
            std::cout << std::endl;
            std::cout << "Bitmap l (size " << this->k_l.size() << ") : ";
    //        for (size_type p = 0; p < k_l.size(); p++) {
    //            std::cout << k_l[p];
    //        }
            std::cout << std::endl;
        }




    protected:

    //*******************************************************//
    //********************** HELPERS ************************//
    //*******************************************************//

    /*! Get the chunk index ([0, k^3[) of a submatrix point.
     *
     * Gets a point in the global matrix and returns its corresponding chunk
     * in the submatrix specified.
     *
     * \param x x coordinate of the point in the global matrix.
     * \param y y coordinate of the point in the global matrix.
     * \param z z coordinate of the point in the global maxtrix.
     * \param x_0 x offset of the submatix in the global matrix.
     * \param y_0 y offset of the submatix in the global matrix.
     * \param z_0 z offset of the submatix in the global matrix.
     * \param l size of the chunk at the submatrix.
     * \param k the k parameter from the k^3 tree.
     * \returns the index of the chunk containing the point at the submatrix.
     */
    inline uint16_t get_chunk_idx(pos_type x, pos_type y, pos_type z,
                                  pos_type x_0, pos_type y_0, pos_type z_0,
                                  size_type l, uint8_t k) const
    {
        return  ((x - x_0) / l) * k * k + ((y - y_0) / l) * k + (z - z_0) / l;
    }

    //*******************************************************//
    //********************* BUILD TREE **********************//
    //*******************************************************//
    //! Build a tree from an point collection
    /*! This method takes a vector of points
     *  and the cube size. And takes linear time over the amount of
     *  points to build the k_3 representation.
     *  \param points A vector with all the points of the cube, it can
     *               not be empty.
     *  \param size Size of the cube, all the nodes in point must be
     *              within 0 and size ([0, size[).
     */
    void build(std::vector<point_type>& points) {

        // Init bit_vectors
        bit_vector_type k_t_ = bit_vector_type(this->k_k1_3 * this->k_height * points.size(), 0);
        bit_vector_type k_l_;

        std::queue<t_part_tuple> q;
        pos_type t = 0, last_level = 0;
        pos_type init_p, end_p, x_0, y_0, z_0, it, t_x, t_y, t_z;
        size_type l = std::pow(this->k_k1, this->k_height - 1);
        std::vector<pos_type > pos_by_chunk(this->k_k1_3 + 1, 0);

        int a = points.size();
        q.push(t_part_tuple(0, points.size(), l, 0, 0, 0));

        while (!q.empty()) {
            std::vector<pos_type> amount_by_chunk(this->k_k1_3, 0);
            std::tie(init_p, end_p, l, x_0, y_0, z_0) = q.front();

            q.pop();

            // Get size for each chunk
            for (it = init_p; it < end_p; it++) {
                amount_by_chunk[get_chunk_idx(points[it].X, points[it].Y, points[it].Z,
                                              x_0, y_0, z_0, l, this->k_k1)] += 1;
            }
            if (l == 1) {
                if (last_level == 0) {
                    last_level = t;
                    k_l_ = bit_vector_type(k_t_.size() - last_level, 0);
                    k_t_.resize(last_level);
                    last_level = 1; // if t was 0
                    t = 0; // Restart counter as we're storing at k_l_ now.
                }
                for (it = 0; it < this->k_k1_3; it++,t++)
                    if (amount_by_chunk[it] != 0)
                        k_l_[t] = 1;
                // At l == 1 we do not put new elements at the queue.
                continue;
            }

            // Set starting position in the vector for each chunk
            pos_by_chunk[0] = init_p;
            for (it = 1; it < this->k_k1_3; it++)
                pos_by_chunk[it] =
                        pos_by_chunk[it - 1] + amount_by_chunk[it - 1];
            // To handle the last case when it = k_2 - 1
            pos_by_chunk[this->k_k1_3] = end_p;
            // Push to the queue every non zero elements chunk
            for (it = 0; it < this->k_k1_3; it++,t++) {
                // If not empty chunk, set bit to 1
                if (amount_by_chunk[it] != 0) {
                    t_x = it / (this->k_k1 * this->k_k1);
                    t_y = (it / this->k_k1) % this->k_k1;
                    t_z = it % this->k_k1;
                    k_t_[t] = 1;
                    q.push(t_part_tuple(pos_by_chunk[it],
                                        pos_by_chunk[it + 1],
                                        l / this->k_k1,
                                        x_0 + t_x * l,
                                        y_0 + t_y * l,
                                        z_0 + t_z * l));
                }
            }
            size_type chunk;

            // Sort edges' vector
            for (unsigned ch = 0; ch < this->k_k1_3; ch++) {
                size_type be = ch == 0 ? init_p : pos_by_chunk[ch - 1];
                for (it = pos_by_chunk[ch]; it < be + amount_by_chunk[ch];) {
                    chunk = get_chunk_idx(points[it].X, points[it].Y, points[it].Z,
                            x_0, y_0, z_0, l, this->k_k1);

                    if (pos_by_chunk[chunk] != it)
                        std::iter_swap(points.begin() + it,
                                       points.begin() + pos_by_chunk[chunk]);
                    else
                        it++;
                    pos_by_chunk[chunk]++;
                }
            }
        }
        k_l_.resize(t);
        //k2_tree_ns::build_template_vector<t_bv>(k_t_, k_l_, k_t, k_l);
        this->k_t = bit_vector_type(k_t_);
        this->k_l = bit_vector_type(k_l_);

        this->k_t_rank = t_rank(&this->k_t);

        //std::cout << "k_t_.size() = " << k_t_.size() << std::endl;
        //for (size_t i = 0; i < k_t_.size(); i++)
        //{
        //    std::cout << "k_t[" << i << "] = " << k_t_[i] << std::endl;
        //}
        //std::cout << "k_l_.size() = " << k_l_.size() << std::endl;
        //for (size_t i = 0; i < k_l_.size(); i++)
        //{
        //    std::cout << "k_l[" << i << "] = " << k_l_[i] << std::endl;
        //}
    }

    //*******************************************************//
    //********************** HELPERS QUERIES ****************//
    //*******************************************************//

    size_type get_region_r(pos_type i_pos_x, pos_type i_pos_y, pos_type i_pos_z,
                           pos_type e_pos_x, pos_type e_pos_y, pos_type e_pos_z,
                           std::vector<point_type> &result,
                           pos_type base_x, pos_type base_y, pos_type base_z,
                           pos_type sub_size, uint16_t level, size_type pos_children) {

        size_type pos;
        uint8_t k = this->k_k1;
        pos_type children_size = sub_size / k;
        pos_type b_x, b_y, b_z;
        size_type n_points = 0;


        // For each child that has cells that overlap with the searched region
        for (pos_type x = i_pos_x/children_size; x <= (e_pos_x/children_size); x++) {
            b_x = x * children_size;
            for (pos_type y = i_pos_y/children_size; y <= (e_pos_y/children_size); y++) {
                b_y = y * children_size;
                for (pos_type z = i_pos_z/children_size; z <= (e_pos_z/children_size); z++) {
                    pos = pos_children + x * k * k + y * k + z; // Position of the current child
                    b_z = z * children_size;


                    if (level < (this->k_height-1)) {
                        // Internal nodes
                        if (this->k_t[pos] == 1) {
                            // Continue with the search process
                            n_points += get_region_r(std::max(b_x, i_pos_x) - b_x,
                                                     std::max(b_y, i_pos_y) - b_y,
                                                     std::max(b_z, i_pos_z) - b_z,
                                                     std::min(b_x + children_size-1, e_pos_x) - b_x,
                                                     std::min(b_y + children_size-1, e_pos_y) - b_y,
                                                     std::min(b_z + children_size-1, e_pos_z) - b_z,
                                                     result,
                                                     base_x + x * children_size, base_y + y * children_size, base_z + z * children_size,
                                                     children_size, level + 1, this->k_t_rank(pos + 1) * this->k_k1_3);
                        }
                    } else {
                        // Leaves nodes
                        if (this->k_l[pos - this->k_t.size()] == 1) {
                            result.push_back({base_x + x, base_y + y, base_z + z});
                            n_points++;
                        }
                    }
                } // END FOR z
            } // END FOR y
        } // END FOR x
        return n_points;
    };

    size_type get_region_r_custom(pos_type i_pos_x, pos_type i_pos_y, pos_type i_pos_z,
                           std::vector<point_type> &result,
                           pos_type base_x, pos_type base_y, pos_type base_z,
                           pos_type sub_size, uint16_t level, size_type pos_children,
                           float &node_size, glm::vec3 &proportional_position) {

        size_type pos;
        uint8_t k = this->k_k1;
        pos_type children_size = sub_size / k;
        pos_type b_x, b_y, b_z;
        size_type n_points = 0;

        // For each child that has cells that overlap with the searched region
        for (pos_type x = i_pos_x/children_size; x <= (i_pos_x/children_size); x++) {
            b_x = x * children_size;
            for (pos_type y = i_pos_y/children_size; y <= (i_pos_y/children_size); y++) {
                b_y = y * children_size;
                for (pos_type z = i_pos_z/children_size; z <= (i_pos_z/children_size); z++) {
                    pos = pos_children + x * k * k + y * k + z; // Position of the current child
                    b_z = z * children_size;


                    if (level < (this->k_height-1)) {
                        // Internal nodes
                        if (this->k_t[pos] == 1) {
                            // Continue with the search process
                            n_points += get_region_r(std::max(b_x, i_pos_x) - b_x,
                                                     std::max(b_y, i_pos_y) - b_y,
                                                     std::max(b_z, i_pos_z) - b_z,
                                                     std::min(b_x + children_size-1, i_pos_x) - b_x,
                                                     std::min(b_y + children_size-1, i_pos_y) - b_y,
                                                     std::min(b_z + children_size-1, i_pos_z) - b_z,
                                                     result,
                                                     base_x + x * children_size, base_y + y * children_size, base_z + z * children_size,
                                                     children_size, level + 1, this->k_t_rank(pos + 1) * this->k_k1_3);
                        }
                    } else {
                        // Leaves nodes
                        if (this->k_l[pos - this->k_t.size()] == 1) {
                            result.push_back({base_x + x, base_y + y, base_z + z});
                            n_points++;
                        }
                    }
                } // END FOR z
            } // END FOR y
        } // END FOR x
        return n_points;
    };

}; // ENC CLASS k3-tree

} // END NAMESPACE sdsl

#endif // K3_TREE_SDSL_K3_TREE
