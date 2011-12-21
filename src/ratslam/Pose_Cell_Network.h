/*
 * This file is part of the RatSLAM C/C++/MATLAB lite versions.
 *
 * This version copyright (C) 2011
 * David Ball (d.ball@itee.uq.edu.au), Scott Heath (scott.heath@uqconnect.edu.au)
 *
 * RatSLAM algorithm by:
 * Michael Milford and Gordon Wyeth ([michael.milford, gordon.wyeth]@qut.edu.au)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * The Pose_Cell_Network class handles continous attractor network dynamics
 * as the cognitive map of a rodent might. A cube of cells is allocated with
 * connections between neighbouring cells. The three dimensions indicate
 * x, y position and facing direction.
 */

#ifndef _POSE_CELL_NETWORK_HPP
#define _POSE_CELL_NETWORK_HPP

#pragma warning( disable: 4275 ) // problem between std::vector and log4cxx
#pragma warning( disable: 4251 ) // problem between std::vector and log4cxx

#define _USE_MATH_DEFINES
#include "math.h"

// todo: replace this with iostream
#include <stdio.h>

#include <boost/property_tree/ini_parser.hpp>
using boost::property_tree::ptree;

typedef double PoseCell;

#include <boost/serialization/access.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/split_member.hpp>

namespace ratslam
{
	
class Pose_Cell_Network
{

public:
	friend class PoseCellsScene;

	Pose_Cell_Network(ptree settings);
	~Pose_Cell_Network();

	// inject energy into a specific point in the network
	bool inject(int act_x, int act_y, int act_z, double energy);

	// locally excite and inhibit points. Excite spreads energy and
	// inhibit compresses.
	bool excite();	
	bool inhibit();	

	// global inhibition
	bool global_inhibit();

	// normalise all the energy in the system
	bool normalise();

	// shift the energy in the system by a
	// translational and rotational velocity.
	bool path_integration(double vtrans, double vrot);

	// find an approximation of the centre of the energy
	// packet.
	double find_best();

	// these updated by find_best()
	double x() { return best_x; }
	double y() { return best_y; }
	double th() { return best_th; }

	// get and set all the cells as one array
	double * get_cells();	
	bool set_cells(double * cells);

	// dump the posecells to a file (not recommended)
	bool dump(const char * filename);

	// access to some of the constants specified in
	// RatSLAM properties.
	double get_delta_pc(double x, double y, double th);
    int get_pc_dim_th() const { return PC_DIM_TH; }
    int get_pc_dim_xy() const { return PC_DIM_XY; }
    double get_pc_global_inhib() const { return PC_GLOBAL_INHIB; }
    int get_pc_w_e_dim() const { return PC_W_E_DIM; }
    double get_pc_w_e_var() const { return PC_W_E_VAR; }
    int get_pc_w_i_dim() const { return PC_W_I_DIM; }
    double get_pc_w_i_var() const { return PC_W_I_VAR; }

	template <typename Archive>
	void save(Archive& ar, const unsigned int version) const
	{
		ar & PC_DIM_XY;
		ar & PC_DIM_TH;
		ar & PC_W_E_DIM;
		ar & PC_W_I_DIM;
		ar & PC_W_E_VAR;
		ar & PC_W_I_VAR;
		ar & PC_GLOBAL_INHIB;

		ar & best_x;
		ar & best_y;
		ar & best_th;

		int i, j, k;
		for (k = 0; k < PC_DIM_TH; k++)
			for (j = 0; j < PC_DIM_XY; j++)
				for (i = 0; i < PC_DIM_XY; i++)
				  ar & posecells[k][j][i];
				
	}

	template <typename Archive>
	void load(Archive& ar, const unsigned int version)
	{
		ar & PC_DIM_XY;
		ar & PC_DIM_TH;
		ar & PC_W_E_DIM;
		ar & PC_W_I_DIM;
		ar & PC_W_E_VAR;
		ar & PC_W_I_VAR;
		ar & PC_GLOBAL_INHIB;
	
		ar & best_x;
		ar & best_y;
		ar & best_th;

		pose_cell_builder();

		int i, j, k;
		for (k = 0; k < PC_DIM_TH; k++)
			for (j = 0; j < PC_DIM_XY; j++)
				for (i = 0; i < PC_DIM_XY; i++)
				  ar & posecells[k][j][i];
	}

	BOOST_SERIALIZATION_SPLIT_MEMBER()
private:
    friend class boost::serialization::access;

	Pose_Cell_Network() {;}
	Pose_Cell_Network(const Pose_Cell_Network & other);
	const Pose_Cell_Network & operator=(const Pose_Cell_Network & other);
	void pose_cell_builder();
	bool pose_cell_excite_helper(int x, int y, int z);
	bool pose_cell_inhibit_helper(int x, int y, int z);
	void circshift2d(double * array, double * array_buffer, int dimx, int dimy, int shiftx, int shifty);
	int rot90_square(double ** array, int dim, int rot);
	int generate_wrap(int * wrap, int start1, int end1, int start2, int end2, int start3, int end3);
	double norm2d(double var, int x, int y, int z, int dim_centre);
	double get_min_delta(double d1, double d2, double max);

	int PC_DIM_XY;
	int PC_DIM_TH;
	int PC_W_E_DIM;
	int PC_W_I_DIM;
	int PC_W_E_VAR;
	int PC_W_I_VAR;
	double PC_GLOBAL_INHIB;

	double best_x;
	double best_y;
	double best_th;

	PoseCell *** posecells;
	PoseCell * posecells_memory;
	int posecells_memory_size;
	int posecells_elements;
	PoseCell *** pca_new;
	PoseCell * pca_new_memory;
	PoseCell ** pca_new_rot_ptr;
	PoseCell ** pca_new_rot_ptr2;
	PoseCell * posecells_plane_th;
	double * PC_W_EXCITE;
	double * PC_W_INHIB;

	int PC_W_E_DIM_HALF;
	int PC_W_I_DIM_HALF;

	int * PC_E_XY_WRAP;
	int * PC_E_TH_WRAP;
	int * PC_I_XY_WRAP;
	int * PC_I_TH_WRAP;

	int PC_CELLS_TO_AVG;
	int * PC_AVG_XY_WRAP;
	int * PC_AVG_TH_WRAP;

	double * PC_XY_SUM_SIN_LOOKUP;
	double * PC_XY_SUM_COS_LOOKUP;
	double * PC_TH_SUM_SIN_LOOKUP;
	double * PC_TH_SUM_COS_LOOKUP;

	double PC_C_SIZE_TH;

};
	
}

#endif // _POSE_CELL_NETWORK_HPP
