/*
 * openRatSLAM
 *
 * utils - General purpose utility helper functions mainly for angles and readings settings
 *
 * Copyright (C) 2012
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * RatSLAM algorithm by:
 * Michael Milford (1) and Gordon Wyeth (1) ([michael.milford, gordon.wyeth]@qut.edu.au)
 *
 * 1. Queensland University of Technology, Australia
 * 2. The University of Queensland, Australia
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
#include "posecell_network.h"
#include "../utils/utils.h"

#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <float.h>

using namespace std;

namespace ratslam
{

PosecellNetwork::PosecellNetwork(ptree settings)
{
  /*
   ** pose cell constants.
   */
  get_setting_from_ptree(PC_DIM_XY, settings, "pc_dim_xy", 21);
  get_setting_from_ptree(PC_DIM_TH, settings, "pc_dim_th", 36);
  get_setting_from_ptree(PC_W_E_DIM, settings, "pc_w_e_dim", 7);
  get_setting_from_ptree(PC_W_I_DIM, settings, "pc_w_i_dim", 5);
  get_setting_from_ptree(PC_W_E_VAR, settings, "pc_w_e_var", 1);
  get_setting_from_ptree(PC_W_I_VAR, settings, "pc_w_i_var", 2);
  get_setting_from_ptree(PC_GLOBAL_INHIB, settings, "pc_global_inhib", 0.00002);

  get_setting_from_ptree(VT_ACTIVE_DECAY, settings, "vt_active_decay", 1.0);
  get_setting_from_ptree(PC_VT_INJECT_ENERGY, settings, "pc_vt_inject_energy", 0.15);
  get_setting_from_ptree(PC_CELL_X_SIZE, settings, "pc_cell_x_size", 1.0);
  get_setting_from_ptree(EXP_DELTA_PC_THRESHOLD, settings, "exp_delta_pc_threshold", 2.0);

  get_setting_from_ptree(PC_VT_RESTORE, settings, "pc_vt_restore", 0.05);

  // the starting position within the posecell network
  best_x = floor((double)PC_DIM_XY / 2.0);
  best_y = floor((double)PC_DIM_XY / 2.0);
  best_th = floor((double)PC_DIM_TH / 2.0);

  current_exp = 0;
  current_vt = 0;

  pose_cell_builder();

  odo_update = false;
  vt_update = false;

}

void PosecellNetwork::pose_cell_builder()
{
  int i, j;

  PC_C_SIZE_TH = (2.0 * M_PI) / PC_DIM_TH;

  // set the sizes
  posecells_memory_size = sizeof(Posecell) * PC_DIM_XY * PC_DIM_XY * PC_DIM_TH;
  posecells_elements = PC_DIM_XY * PC_DIM_XY * PC_DIM_TH;

  // allocate the memory
  posecells_memory = (Posecell *)malloc((size_t)posecells_memory_size);
  pca_new_memory = (Posecell *)malloc((size_t)posecells_memory_size);

  // zero all the memory
  memset(posecells_memory, 0, (size_t)posecells_memory_size);
  memset(pca_new_memory, 0, (size_t)posecells_memory_size);

  // allocate first level pointers
  posecells = (Posecell ***)malloc(sizeof(Posecell**) * PC_DIM_TH);
  pca_new = (Posecell ***)malloc(sizeof(Posecell**) * PC_DIM_TH);

  for (i = 0; i < PC_DIM_TH; i++)
  {
    // allocate second level pointers
    posecells[i] = (Posecell **)malloc(sizeof(Posecell*) * PC_DIM_XY);
    pca_new[i] = (Posecell **)malloc(sizeof(Posecell*) * PC_DIM_XY);

    for (j = 0; j < PC_DIM_XY; j++)
    {
      // TRICKY! point second level pointers at already allocated memory
      posecells[i][j] = &posecells_memory[(i * PC_DIM_XY + j) * PC_DIM_XY];
      pca_new[i][j] = &pca_new_memory[(i * PC_DIM_XY + j) * PC_DIM_XY];
    }
  }

  // for path integration
  pca_new_rot_ptr = (Posecell **)malloc(sizeof(Posecell*) * (PC_DIM_XY + 2));
  pca_new_rot_ptr2 = (Posecell **)malloc(sizeof(Posecell*) * (PC_DIM_XY + 2));
  for (i = 0; i < PC_DIM_XY + 2; i++)
  {
    pca_new_rot_ptr[i] = &pca_new_memory[(i * (PC_DIM_XY + 2))];
    pca_new_rot_ptr2[i] = &pca_new_memory[(PC_DIM_XY + 2) * (PC_DIM_XY + 2) + (i * (PC_DIM_XY + 2))];
  }

  posecells_plane_th = (Posecell *)malloc(sizeof(Posecell) * (PC_DIM_XY + 2) * (PC_DIM_XY + 2));

  PC_W_EXCITE = (double *)malloc(sizeof(double) * PC_W_E_DIM * PC_W_E_DIM * PC_W_E_DIM);
  PC_W_INHIB = (double *)malloc(sizeof(double) * PC_W_I_DIM * PC_W_I_DIM * PC_W_I_DIM);

  posecells[(int)best_th][(int)best_y][(int)best_x] = 1;

  // set up the wrap lookups
  PC_W_E_DIM_HALF = (int)floor((double)PC_W_E_DIM / 2.0);
  PC_W_I_DIM_HALF = (int)floor((double)PC_W_I_DIM / 2.0);

  PC_E_XY_WRAP = (int *)malloc((PC_DIM_XY + PC_W_E_DIM - 1) * sizeof(int));
  PC_E_TH_WRAP = (int *)malloc((PC_DIM_TH + PC_W_E_DIM - 1) * sizeof(int));
  PC_I_XY_WRAP = (int *)malloc((PC_DIM_XY + PC_W_I_DIM - 1) * sizeof(int));
  PC_I_TH_WRAP = (int *)malloc((PC_DIM_TH + PC_W_I_DIM - 1) * sizeof(int));

  generate_wrap(PC_E_XY_WRAP, PC_DIM_XY - PC_W_E_DIM_HALF, PC_DIM_XY, 0, PC_DIM_XY, 0, PC_W_E_DIM_HALF);
  generate_wrap(PC_E_TH_WRAP, PC_DIM_TH - PC_W_E_DIM_HALF, PC_DIM_TH, 0, PC_DIM_TH, 0, PC_W_E_DIM_HALF);
  generate_wrap(PC_I_XY_WRAP, PC_DIM_XY - PC_W_I_DIM_HALF, PC_DIM_XY, 0, PC_DIM_XY, 0, PC_W_I_DIM_HALF);
  generate_wrap(PC_I_TH_WRAP, PC_DIM_TH - PC_W_I_DIM_HALF, PC_DIM_TH, 0, PC_DIM_TH, 0, PC_W_I_DIM_HALF);

  PC_CELLS_TO_AVG = 3;
  PC_AVG_XY_WRAP = (int *)malloc((PC_DIM_XY + 2 * PC_CELLS_TO_AVG) * sizeof(int));
  PC_AVG_TH_WRAP = (int *)malloc((PC_DIM_TH + 2 * PC_CELLS_TO_AVG) * sizeof(int));

  generate_wrap(PC_AVG_XY_WRAP, PC_DIM_XY - PC_CELLS_TO_AVG, PC_DIM_XY, 0, PC_DIM_XY, 0, PC_CELLS_TO_AVG);
  generate_wrap(PC_AVG_TH_WRAP, PC_DIM_TH - PC_CELLS_TO_AVG, PC_DIM_TH, 0, PC_DIM_TH, 0, PC_CELLS_TO_AVG);

  // sine and cosine lookups
  PC_XY_SUM_SIN_LOOKUP = (double *)malloc(PC_DIM_XY * sizeof(double));
  PC_XY_SUM_COS_LOOKUP = (double *)malloc(PC_DIM_XY * sizeof(double));
  PC_TH_SUM_SIN_LOOKUP = (double *)malloc(PC_DIM_TH * sizeof(double));
  PC_TH_SUM_COS_LOOKUP = (double *)malloc(PC_DIM_TH * sizeof(double));

  for (i = 0; i < PC_DIM_XY; i++)
  {
    PC_XY_SUM_SIN_LOOKUP[i] = sin((double)(i + 1) * 2.0 * M_PI / (double)PC_DIM_XY);
    PC_XY_SUM_COS_LOOKUP[i] = cos((double)(i + 1) * 2.0 * M_PI / (double)PC_DIM_XY);
  }

  for (i = 0; i < PC_DIM_TH; i++)
  {
    PC_TH_SUM_SIN_LOOKUP[i] = sin((double)(i + 1) * 2.0 * M_PI / (double)PC_DIM_TH);
    PC_TH_SUM_COS_LOOKUP[i] = cos((double)(i + 1) * 2.0 * M_PI / (double)PC_DIM_TH);
  }

  double total = 0;
  int k, next = 0;
  int dim_centre = PC_W_E_DIM / 2;

  for (k = 0; k < PC_W_E_DIM; k++)
  {
    for (j = 0; j < PC_W_E_DIM; j++)
    {
      for (i = 0; i < PC_W_E_DIM; i++)
      {
        PC_W_EXCITE[next] = norm2d(PC_W_E_VAR, i, j, k, dim_centre);
        total += PC_W_EXCITE[next];
        next++;
      }
    }
  }

  for (next = 0; next < PC_W_E_DIM * PC_W_E_DIM * PC_W_E_DIM; next++)
  {
    PC_W_EXCITE[next] /= total;
  }

  total = 0;
  dim_centre = PC_W_I_DIM / 2;
  next = 0;
  for (k = 0; k < PC_W_I_DIM; k++)
  {
    for (j = 0; j < PC_W_I_DIM; j++)
    {
      for (i = 0; i < PC_W_I_DIM; i++)
      {
        PC_W_INHIB[next] = norm2d(PC_W_I_VAR, i, j, k, dim_centre);
        total += PC_W_INHIB[next];
        next++;
      }
    }
  }

  for (next = 0; next < PC_W_I_DIM * PC_W_I_DIM * PC_W_I_DIM; next++)
  {
    PC_W_INHIB[next] /= total;
  }
}

PosecellNetwork::~PosecellNetwork()
{
  int i;
  for (i = 0; i < PC_DIM_TH; i++)
  {
    free(posecells[i]);
    free(pca_new[i]);
  }
  free(posecells);
  free(pca_new);
  free(pca_new_rot_ptr);
  free(pca_new_rot_ptr2);
  free(PC_W_EXCITE);
  free(PC_W_INHIB);
  free(PC_E_XY_WRAP);
  free(PC_I_XY_WRAP);
  free(PC_E_TH_WRAP);
  free(PC_I_TH_WRAP);
  free(PC_AVG_XY_WRAP);
  free(PC_AVG_TH_WRAP);
  free(PC_XY_SUM_SIN_LOOKUP);
  free(PC_XY_SUM_COS_LOOKUP);
  free(PC_TH_SUM_SIN_LOOKUP);
  free(PC_TH_SUM_COS_LOOKUP);
  free(posecells_memory);
  free(pca_new_memory);
}

bool PosecellNetwork::inject(int act_x, int act_y, int act_z, double energy)
{

  if (act_x < PC_DIM_XY && act_x >= 0 && act_y < PC_DIM_XY && act_y >= 0 && act_z < PC_DIM_TH && act_z >= 0)
    posecells[act_z][act_y][act_x] += energy;

  return true;
}

bool PosecellNetwork::excite(void)
{
  int i, j, k;

  // set all of pca_new to 0
  memset(pca_new_memory, 0, posecells_memory_size);

  // loop in all three dimensions
  for (i = 0; i < PC_DIM_XY; i++)
  {
    for (j = 0; j < PC_DIM_XY; j++)
    {
      for (k = 0; k < PC_DIM_TH; k++)
      {
        if (posecells[k][j][i] != 0)
        {
          // spread the pose cell energy
          pose_cell_excite_helper(i, j, k);
        }
      }
    }
  }

  //  pc.Posecells = pca_new;
  memcpy(posecells_memory, pca_new_memory, posecells_memory_size);
  return true;

}

bool PosecellNetwork::inhibit(void)
{
  int i, j, k;

  // set pca_new to 0
  memset(pca_new_memory, 0, posecells_memory_size);

  // loop through all dimensions
  for (i = 0; i < PC_DIM_XY; i++)
  {
    for (j = 0; j < PC_DIM_XY; j++)
    {
      for (k = 0; k < PC_DIM_TH; k++)
      {
        if (posecells[k][j][i] != 0)
        {
          // if there is energy in the current posecell,
          // spread the energy
          pose_cell_inhibit_helper(i, j, k);
        }
      }
    }
  }

  for (i = 0; i < posecells_elements; i++)
  {
    posecells_memory[i] -= pca_new_memory[i];
  }

  return true;
}

bool PosecellNetwork::global_inhibit()
{
  int i;
  for (i = 0; i < posecells_elements; i++)
  {
    if (posecells_memory[i] >= PC_GLOBAL_INHIB)
    {
      posecells_memory[i] = (posecells_memory[i] - PC_GLOBAL_INHIB);
    }
    else
    {
      posecells_memory[i] = 0;
    }
  }
  return true;
}

bool PosecellNetwork::normalise(void)
{
  int i;
  double total = 0;
  for (i = 0; i < posecells_elements; i++)
  {
    total += posecells_memory[i];
  }

  assert(total > 0);

  for (i = 0; i < posecells_elements; i++)
  {
    posecells_memory[i] /= total;
    //assert(posecells_memory[i] >= 0);
    //assert(!isnan(posecells_memory[i]));
  }

  return true;

}

bool PosecellNetwork::path_integration(double vtrans, double vrot)
{
  int dir_pc;
  double angle_to_add = 0;

  // scaling
  vtrans /= PC_CELL_X_SIZE;

  if (vtrans < 0)
  {
    vtrans = -vtrans;
    angle_to_add = M_PI;
  }
  // % shift in each th given by the th
  // for dir_pc=1:PARAMS.PC_DIM_TH
  for (dir_pc = 0; dir_pc < PC_DIM_TH; dir_pc++)
  {

    // % radians
    // dir = (dir_pc - 1) * pc.PC_C_SIZE_TH;
    double dir = dir_pc * PC_C_SIZE_TH + angle_to_add;

    double dir90, weight_sw, weight_se, weight_nw, weight_ne;
    int i, j;

    // % rotate the pc.Posecells instead of implementing for four quadrants
    // pca90 = rot90(pc.Posecells(:,:,dir_pc), floor(dir *2/pi));
    rot90_square(posecells[dir_pc], PC_DIM_XY, (int)floor(dir * 2.0 / M_PI));

    // dir90 = dir - floor(dir *2/pi) * pi/2;
    dir90 = dir - floor(dir * 2 / M_PI) * M_PI / 2;

    // % extend the pc.Posecells one unit in each direction (max supported at the moment)
    // % work out the weight contribution to the NE cell from the SW, NW, SE cells
    // % given vtrans and the direction
    // % weight_sw = v * cos(th) * v * sin(th)
    // % weight_se = (1 - v * cos(th)) * v * sin(th)
    // % weight_nw = (1 - v * sin(th)) * v * sin(th)
    // % weight_ne = 1 - weight_sw - weight_se - weight_nw
    // % think in terms of NE divided into 4 rectangles with the sides
    // % given by vtrans and the angle
    // pca_new=zeros(PARAMS.PC_DIM_XY+2);
    memset(pca_new_memory, 0, sizeof(double) * (PC_DIM_XY + 2) * (PC_DIM_XY + 2));

    // pca_new(2:end-1,2:end-1) = pca90;
    for (j = 0; j < PC_DIM_XY; j++)
    {
      memcpy(&pca_new_rot_ptr[j + 1][1], &posecells[dir_pc][j][0], sizeof(double) * PC_DIM_XY);
    }

    // weight_sw = vtrans^2 *cos(dir90) * sin(dir90);
    weight_sw = vtrans * vtrans * cos(dir90) * sin(dir90);

    // weight_se = vtrans*sin(dir90) - vtrans^2 *cos(dir90) * sin(dir90);
    weight_se = //vtrans*sin(dir90) - vtrans*vtrans*cos(dir90)*sin(dir90);
        vtrans * sin(dir90) * (1.0 - vtrans * cos(dir90));

    // weight_nw = vtrans*cos(dir90) - vtrans^2 *cos(dir90) * sin(dir90);
    weight_nw = // vtrans*cos(dir90) - vtrans*vtrans*cos(dir90)*sin(dir90);
        vtrans * cos(dir90) * (1.0 - vtrans * sin(dir90));

    // weight_ne = 1.0 - weight_sw - weight_se - weight_nw;
    weight_ne = 1.0 - weight_sw - weight_se - weight_nw;

    /* if (weight_sw < 0 || weight_se < 0 || weight_nw < 0 || weight_ne < 0)
     {
     printf("WARNING: weights are negative, vtrans(%f) is either negative or too big\n", vtrans);
     printf("WARNING: continuing, but expect possible failures soon! Update POSECELL_VTRANS_SCALING to fix this.\n", vtrans);
     }*/

    // % circular shift and multiple by the contributing weight
    // % copy those shifted elements for the wrap around
    // pca_new = pca_new.*weight_ne + [pca_new(:,end) pca_new(:,1:end-1)].*weight_nw + [pca_new(end,:); pca_new(1:end-1,:)].*weight_se + circshift(pca_new, [1 1]).*weight_sw;
    // first element
    pca_new_rot_ptr2[0][0] = pca_new_rot_ptr[0][0] * weight_ne + pca_new_rot_ptr[0][PC_DIM_XY + 1] * weight_se + pca_new_rot_ptr[PC_DIM_XY + 1][0] * weight_nw;

    // first row
    for (i = 1; i < PC_DIM_XY + 2; i++)
    {
      pca_new_rot_ptr2[0][i] = pca_new_rot_ptr[0][i] * weight_ne + pca_new_rot_ptr[0][i - 1] * weight_se + pca_new_rot_ptr[PC_DIM_XY + 1][i] * weight_nw;
    }

    for (j = 1; j < PC_DIM_XY + 2; j++)
    {
      // first column
      pca_new_rot_ptr2[j][0] = pca_new_rot_ptr[j][0] * weight_ne + pca_new_rot_ptr[j][PC_DIM_XY + 1] * weight_se + pca_new_rot_ptr[j - 1][0] * weight_nw;

      // all the rest
      for (i = 1; i < PC_DIM_XY + 2; i++)
      {
        pca_new_rot_ptr2[j][i] = pca_new_rot_ptr[j][i] * weight_ne + pca_new_rot_ptr[j][i - 1] * weight_se + pca_new_rot_ptr[j - 1][i] * weight_nw;
      }
    }

    circshift2d(pca_new_rot_ptr[0], posecells_plane_th, PC_DIM_XY + 2, PC_DIM_XY + 2, 1, 1);

    for (i = 0; i < (PC_DIM_XY + 2) * (PC_DIM_XY + 2); i++)
    {
      pca_new_rot_ptr2[0][i] += pca_new_rot_ptr[0][i] * weight_sw;
    }

    // pca90 = pca_new(2:end-1,2:end-1);
    for (j = 0; j < PC_DIM_XY; j++)
    {
      for (i = 0; i < PC_DIM_XY; i++)
      {
        posecells[dir_pc][j][i] = pca_new_rot_ptr2[j + 1][i + 1];
      }
    }

    // pca90(2:end,1) = pca90(2:end,1) + pca_new(3:end-1,end);
    for (i = 1; i < PC_DIM_XY; i++)
    {
      posecells[dir_pc][0][i] += pca_new_rot_ptr2[PC_DIM_XY + 1][i + 1];
    }

    // pca90(1,2:end) = pca90(1,2:end) + pca_new(end,3:end-1);
    for (j = 1; j < PC_DIM_XY; j++)
    {
      posecells[dir_pc][j][0] += pca_new_rot_ptr2[j + 1][PC_DIM_XY + 1];
    }

    // pca90(1,1) = pca90(1,1) + pca_new(end:end);
    posecells[dir_pc][0][0] += pca_new_rot_ptr2[PC_DIM_XY + 1][PC_DIM_XY + 1];

    // % unrotate the pose cell xy layer
    // pc.Posecells(:,:,dir_pc) = rot90(pca90, 4 - floor(dir * 2/pi));
    rot90_square(posecells[dir_pc], PC_DIM_XY, 4 - (int)floor(dir * 2.0 / M_PI));
    //end

    // end
  }

  // % Path Integration - Theta
  // % Shift the pose cells +/- theta given by vrot
  // if vrot ~= 0
  if (vrot != 0)
  {
    int i, j, k;
    double weight;
    double sign_vrot;
    int shifty1, shifty2;
    int newk1, newk2;

    // % mod to work out the partial shift amount
    // weight = mod(abs(vrot)/pc.PC_C_SIZE_TH, 1);
    weight = abs(vrot) / PC_C_SIZE_TH;

    while (weight > 1)
    {
      weight -= 1;
    }

    // if weight == 0
    if (weight == 0)
    {
      //     weight = 1.0;
      weight = 1.0;
    }
    // end

    memcpy(pca_new_memory, posecells_memory, posecells_memory_size);
    sign_vrot = vrot < 0 ? -1 : 1;
    shifty1 = (int)(sign_vrot * floor(abs(vrot) / PC_C_SIZE_TH));
    shifty2 = (int)(sign_vrot * ceil(abs(vrot) / PC_C_SIZE_TH));

    while (shifty1 > 0)
    {
      shifty1 -= PC_DIM_TH;
    }

    while (shifty2 > 0)
    {
      shifty2 -= PC_DIM_TH;
    }

    for (j = 0; j < PC_DIM_XY; j++)
    {
      for (i = 0; i < PC_DIM_XY; i++)
      {
        for (k = 0; k < PC_DIM_TH; k++)
        {
          newk1 = (k - shifty1) % PC_DIM_TH;
          newk2 = (k - shifty2) % PC_DIM_TH;
          assert(newk1 < PC_DIM_TH);
          assert(newk2 < PC_DIM_TH);
          assert(newk1 >= 0);
          assert(newk2 >= 0);
          posecells[k][j][i] = pca_new[newk1][j][i] * (1.0 - weight) + pca_new[newk2][j][i] * weight;
          // assert(posecells[k][j][i] >= 0);
        }
      }

    }
    // end
  }

  return true;
}

double PosecellNetwork::find_best()
{
  int i, j, k;
  double x = -1, y = -1, th = -1;

  double * x_sums, *y_sums, *z_sums;
  double sum_x1, sum_x2, sum_y1, sum_y2;

  // % find the max activated cell
  double max = 0;
  for (k = 0; k < PC_DIM_TH; k++)
  {
    for (j = 0; j < PC_DIM_XY; j++)
    {
      for (i = 0; i < PC_DIM_XY; i++)
      {
        if (posecells[k][j][i] > max)
        {
          max = posecells[k][j][i];
          x = (double)i;
          y = (double)j;
          th = (double)k;
        }
      }
    }
  }

  //  % take the max activated cell +- AVG_CELL in 3d space
  //  % get the sums for each axis
  memset(pca_new_memory, 0, posecells_memory_size);

  x_sums = pca_new[0][0];
  y_sums = pca_new[1][0];
  z_sums = pca_new[2][0];

  for (k = (int)th; k < th + PC_CELLS_TO_AVG * 2 + 1; k++)
  {
    for (j = (int)y; j < y + PC_CELLS_TO_AVG * 2 + 1; j++)
    {
      for (i = (int)x; i < x + PC_CELLS_TO_AVG * 2 + 1; i++)
      {
        // pca_new[PC_AVG_TH_WRAP[k]][PC_AVG_XY_WRAP[j]][PC_AVG_XY_WRAP[i]] =
        //  posecells[PC_AVG_TH_WRAP[k]][PC_AVG_XY_WRAP[j]][PC_AVG_XY_WRAP[i]];

        z_sums[PC_AVG_TH_WRAP[k]] += posecells[PC_AVG_TH_WRAP[k]][PC_AVG_XY_WRAP[j]][PC_AVG_XY_WRAP[i]];
        y_sums[PC_AVG_XY_WRAP[j]] += posecells[PC_AVG_TH_WRAP[k]][PC_AVG_XY_WRAP[j]][PC_AVG_XY_WRAP[i]];
        x_sums[PC_AVG_XY_WRAP[i]] += posecells[PC_AVG_TH_WRAP[k]][PC_AVG_XY_WRAP[j]][PC_AVG_XY_WRAP[i]];
      }
    }
  }

  //  % now find the (x, y, th) using population vector decoding to handle the wrap around
  sum_x1 = 0;
  sum_x2 = 0;
  sum_y1 = 0;
  sum_y2 = 0;
  for (i = 0; i < PC_DIM_XY; i++)
  {
    sum_x1 += PC_XY_SUM_SIN_LOOKUP[i] * x_sums[i];
    sum_x2 += PC_XY_SUM_COS_LOOKUP[i] * x_sums[i];
    sum_y1 += PC_XY_SUM_SIN_LOOKUP[i] * y_sums[i];
    sum_y2 += PC_XY_SUM_COS_LOOKUP[i] * y_sums[i];
  }

  x = atan2(sum_x1, sum_x2) * (PC_DIM_XY) / (2.0 * M_PI) - 1.0;
  while (x < 0)
  {
    x += PC_DIM_XY;
  }
  while (x > PC_DIM_XY)
  {
    x -= PC_DIM_XY;
  }

  y = atan2(sum_y1, sum_y2) * (PC_DIM_XY) / (2.0 * M_PI) - 1.0;
  while (y < 0)
  {
    y += PC_DIM_XY;
  }

  while (y > PC_DIM_XY)
  {
    y -= PC_DIM_XY;
  }

  sum_x1 = 0;
  sum_x2 = 0;
  for (i = 0; i < PC_DIM_TH; i++)
  {
    sum_x1 += PC_TH_SUM_SIN_LOOKUP[i] * z_sums[i];
    sum_x2 += PC_TH_SUM_COS_LOOKUP[i] * z_sums[i];
  }
  th = atan2(sum_x1, sum_x2) * (PC_DIM_TH) / (2.0 * M_PI) - 1.0;
  while (th < 0)
  {
    th += PC_DIM_TH;
  }
  while (th > PC_DIM_TH)
  {
    th -= PC_DIM_TH;
  }

  if (x < 0 || y < 0 || th < 0 || x > PC_DIM_XY || y > PC_DIM_XY || th > PC_DIM_TH)
  {
    cout << "ERROR: " << x << ", " << y << ", " << th << " out of range" << endl;
  }

  best_x = x;
  best_y = y;
  best_th = th;

  return max;
}

double * PosecellNetwork::get_cells(void)
{
  return posecells_memory;
}

bool PosecellNetwork::set_cells(double * cells)
{
  if (memcpy(posecells_memory, cells, posecells_memory_size) != NULL)
    return true;
  else
    return false;
}

double PosecellNetwork::get_delta_pc(double x, double y, double th)
{
  double pc_th_corrected = best_th - vt_delta_pc_th;
  if (pc_th_corrected < 0) 
	pc_th_corrected = PC_DIM_TH + pc_th_corrected;
  if (pc_th_corrected >= PC_DIM_TH)
	pc_th_corrected = pc_th_corrected - PC_DIM_TH;
  return sqrt(pow(get_min_delta(best_x, x, PC_DIM_XY), 2) + pow(get_min_delta(best_y, y, PC_DIM_XY), 2) + pow(get_min_delta(pc_th_corrected, th, PC_DIM_TH), 2));
}

double PosecellNetwork::get_min_delta(double d1, double d2, double max)
{
  double absval = abs(d1 - d2);
  return min(absval, max - absval);
}

bool PosecellNetwork::pose_cell_excite_helper(int x, int y, int z)
{
  int xl, yl, zl, xw, yw, zw, excite_index = 0;

  // loop in all dimensions
  for (zl = z; zl < z + PC_W_E_DIM; zl++)
  {
    for (yl = y; yl < y + PC_W_E_DIM; yl++)
    {
      for (xl = x; xl < x + PC_W_E_DIM; xl++)
      {
        // generate indices by wrapping where necessary
        xw = PC_E_XY_WRAP[xl];
        yw = PC_E_XY_WRAP[yl];
        zw = PC_E_TH_WRAP[zl];

        // for every pose cell, multiply the current energy by
        // a pdf to spread the energy (PC_W_EXCITE is a 3d pdf)
        pca_new[zw][yw][xw] += posecells[z][y][x] * PC_W_EXCITE[excite_index++];
      }
    }
  }

  return true;
}

bool PosecellNetwork::pose_cell_inhibit_helper(int x, int y, int z)
{
  int xl, yl, zl, xw, yw, zw, inhib_index = 0;

  // loop through all the dimensions
  for (zl = z; zl < z + PC_W_I_DIM; zl++)
  {
    for (yl = y; yl < y + PC_W_I_DIM; yl++)
    {
      for (xl = x; xl < x + PC_W_I_DIM; xl++)
      {
        // generate indices by wrapping where necessary
        xw = PC_I_XY_WRAP[xl];
        yw = PC_I_XY_WRAP[yl];
        zw = PC_I_TH_WRAP[zl];

        // spread the energy using a pdf
        pca_new[zw][yw][xw] += posecells[z][y][x] * PC_W_INHIB[inhib_index++];
      }
    }
  }

  return true;
}

void PosecellNetwork::circshift2d(double * array, double * array_buffer, int dimx, int dimy, int shiftx, int shifty)
{
  if (shifty == 0)
  {
    if (shiftx == 0)
    {
      return;
    }

    memcpy(array_buffer, array, dimx * dimy * sizeof(double));
  }
  else if (shifty > 0)
  {
    memcpy(array_buffer, &array[(dimy - shifty) * dimx], shifty * dimx * sizeof(double));
    memcpy(&array_buffer[shifty * dimx], array, (dimy - shifty) * dimx * sizeof(double));
  }
  else
  {
    memcpy(array_buffer, &array[-shifty * dimx], (dimy + shifty) * dimx * sizeof(double));
    memcpy(&array_buffer[(dimy + shifty) * dimx], array, -shifty * dimx * sizeof(double));
  }

  if (shiftx == 0)
  {
    memcpy(array, array_buffer, dimx * dimy * sizeof(double));
  }
  else if (shiftx > 0)
  {
    int i;
    for (i = 0; i < dimy; i++)
    {
      memcpy(&array[i * dimx], &array_buffer[i * dimx + dimx - shiftx], shiftx * sizeof(double));
      memcpy(&array[i * dimx + shiftx], &array_buffer[i * dimx], (dimx - shiftx) * sizeof(double));
    }
  }
  else
  {
    int i;
    for (i = 0; i < dimy; i++)
    {
      memcpy(&array[i * dimx], &array_buffer[i * dimx - shiftx], (dimx + shiftx) * sizeof(double));
      memcpy(&array[i * dimx + dimx + shiftx], &array_buffer[i * dimx], -shiftx * sizeof(double));
    }
  }
}

int PosecellNetwork::rot90_square(double ** array, int dim, int rot)
{
  double centre = (double)(dim - 1) / 2.0f;

  double a, b, c, d;

  double tmp_new, tmp_old;

  int i, j, quad, id, jd, is1, js1;

  if (rot < 0)
  {
    rot += 4;
  }

  switch (rot % 4)
  {
    case 0:
      return 1;
    case 1:
      a = 0;
      b = -1;
      c = 1;
      d = 0;
      break;
    case 2:
      a = -1;
      b = 0;
      c = 0;
      d = -1;
      break;
    case 3:
      a = 0;
      b = 1;
      c = -1;
      d = 0;
      break;
    default:
	return 1;
  }

  if (rot % 2 == 1)
  {
    for (j = 0; j < (int)(centre) + (1 - dim % 2); j++)
    {
      for (i = 0; i < (int)centre + 1; i++)
      {
        id = i;
        jd = j;
        tmp_old = array[j][i];
        for (quad = 0; quad < 4; quad++)
        {
          is1 = id;
          js1 = jd;
          id = (int)(a * ((float)(is1) - centre) + b * ((float)(js1) - centre) + centre);
          jd = (int)(c * ((float)(is1) - centre) + d * ((float)(js1) - centre) + centre);
          tmp_new = array[jd][id];
          array[jd][id] = tmp_old;
          tmp_old = tmp_new;
        }
      }
    }
  }
  else
  {
    rot90_square(array, dim, 1);
    rot90_square(array, dim, 1);
  }
  return true;
}

int PosecellNetwork::generate_wrap(int * wrap, int start1, int end1, int start2, int end2, int start3, int end3)
{
  int i, j;
  i = 0;
  for (j = start1; j < end1; i++, j++)
  {
    wrap[i] = j;
  }

  for (j = start2; j < end2; i++, j++)
  {
    wrap[i] = j;
  }

  for (j = start3; j < end3; i++, j++)
  {
    wrap[i] = j;
  }
  return 1;
}

double PosecellNetwork::norm2d(double var, int x, int y, int z, int dim_centre)
{
  return 1.0 / (var * sqrt(2.0 * M_PI))
      * exp((-(x - dim_centre) * (x - dim_centre) - (y - dim_centre) * (y - dim_centre) - (z - dim_centre) * (z - dim_centre)) / (2.0 * var * var));
}

void PosecellNetwork::create_experience()
{
  PosecellVisualTemplate * pcvt = &visual_templates[current_vt];
  experiences.resize(experiences.size() + 1);
  current_exp = experiences.size() - 1;
  PosecellExperience * exp = &experiences[current_exp];
  exp->x_pc = x();
  exp->y_pc = y();
  exp->th_pc = th();
  exp->vt_id = current_vt;
  pcvt->exps.push_back(current_exp);
}


PosecellNetwork::PosecellAction PosecellNetwork::get_action()
{
  PosecellExperience * experience;
  double delta_pc;
  PosecellAction action = NO_ACTION;

  if (odo_update && vt_update)
  {
    odo_update = false;
    vt_update = false;

  } else {
	return action;
  }

  if (visual_templates.size() == 0)
  {
    action = NO_ACTION;
    return action;
  }

  if (experiences.size() == 0)
  {
    create_experience();
    action = CREATE_NODE;
  } else {
    experience = &experiences[current_exp];

    delta_pc = get_delta_pc(experience->x_pc, experience->y_pc, experience->th_pc);

    PosecellVisualTemplate * pcvt = &visual_templates[current_vt];
    if (pcvt->exps.size() == 0)
    {
      create_experience();
      action = CREATE_NODE;
     }
    else if (delta_pc > EXP_DELTA_PC_THRESHOLD || current_vt != prev_vt)
    {
      // go through all the exps associated with the current view and find the one with the closest delta_pc
      int matched_exp_id = -1;
      unsigned int i;
      int min_delta_id = -1;
      double min_delta = DBL_MAX;
      double delta_pc;

    // find the closest experience in pose cell space
      for (i = 0; i < pcvt->exps.size(); i++)
      {
        // make sure we aren't comparing to the current experience
        if (current_exp == pcvt->exps[i])
          continue;

        experience = &experiences[pcvt->exps[i]];
        delta_pc = get_delta_pc(experience->x_pc, experience->y_pc, experience->th_pc);

        if (delta_pc < min_delta)
        {
          min_delta = delta_pc;
          min_delta_id = pcvt->exps[i];
        }
      }

      // if an experience is closer than the thres create a link
      if (min_delta < EXP_DELTA_PC_THRESHOLD)
      {
        matched_exp_id = min_delta_id;
        action = CREATE_EDGE;
      }

      if (current_exp != (unsigned)matched_exp_id)
      {
        if (matched_exp_id == -1)
        {
          create_experience();
          action = CREATE_NODE;
        }
        else
        {
          current_exp = matched_exp_id;
          if (action == NO_ACTION)
          {
            action = SET_NODE;
          }
        }
      }
      else if (current_vt == prev_vt)
      {
        create_experience();
        action = CREATE_NODE;
      }
    }
  }

  return action;
}

void PosecellNetwork::on_odo(double vtrans, double vrot, double time_diff_s)
{
  vtrans = vtrans * time_diff_s;
  vrot = vrot * time_diff_s;

  excite();
  inhibit();
  global_inhibit();
  normalise();
  path_integration(vtrans, vrot);
  find_best();
  odo_update = true;
}

void PosecellNetwork::create_view_template()
{
  PosecellVisualTemplate * pcvt;
  visual_templates.resize(visual_templates.size() + 1);
  pcvt = &visual_templates[visual_templates.size() - 1];
  pcvt->pc_x = x();
  pcvt->pc_y = y();
  pcvt->pc_th = th();
  pcvt->decay = VT_ACTIVE_DECAY;

}

void PosecellNetwork::on_view_template(unsigned int vt, double vt_rad)
{
  PosecellVisualTemplate * pcvt;
  if (vt >= visual_templates.size())
  {
    // must be a new template
    create_view_template();
    assert(vt == visual_templates.size()-1);
  }
  else
  {
    // the template must exist
    pcvt = &visual_templates[vt];

    // this prevents energy injected in recently created vt's
    if (vt < (visual_templates.size() - 10))
    {
      if (vt != current_vt)
      {
      } else {
        pcvt->decay += VT_ACTIVE_DECAY;
      }

      // this line is magic. ask michael about it
      double energy = PC_VT_INJECT_ENERGY * 1.0 / 30.0 * (30.0 - exp(1.2 * pcvt->decay));
      if (energy > 0)
      {
		vt_delta_pc_th = vt_rad / (2.0*M_PI) * PC_DIM_TH;
		double pc_th_corrected = pcvt->pc_th + vt_rad / (2.0*M_PI) * PC_DIM_TH;
		if (pc_th_corrected < 0) 
			pc_th_corrected = PC_DIM_TH + pc_th_corrected;
		if (pc_th_corrected >= PC_DIM_TH)
			pc_th_corrected = pc_th_corrected - PC_DIM_TH;
        inject((int)pcvt->pc_x, (int)pcvt->pc_y, (int)pc_th_corrected, energy);
      }
    }
  }

  for (unsigned int i=0; i < visual_templates.size(); i++)
  {
    visual_templates[i].decay -= PC_VT_RESTORE;
    if (visual_templates[i].decay < VT_ACTIVE_DECAY)
      visual_templates[i].decay = VT_ACTIVE_DECAY;
  }

  prev_vt = current_vt;
  current_vt = vt;

vt_update = true;
}

} // namespace ratslam

