/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

using namespace std;
using namespace octomap;
using namespace octomath;

void print_query_info(point3d query, OcTreeNode* node) {
  if (node != NULL) {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
  }
  else 
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;    
}

int main(int argc, char** argv) {

  cout << endl;
  cout << "generating example map" << endl;

  float resolution_value = 0.1;
  float step_value = resolution_value / 2.0f;
  OcTree tree (resolution_value);  // create empty tree with resolution 0.1
  Pose6D rot = Pose6D(0,0,0,0,0,M_PI/6.0f);

  // insert some measurements of occupied cells

  // Truck's roof above drivers
  int x_min = - (int)(1.0f/resolution_value);
  int x_max = - x_min;
  int y_min = - (int)(1.5f/resolution_value);
  int y_max = - y_min;
  int z_min = - (int)(1.0f/resolution_value);
  int z_max = - z_min;

  cout << "Roof's size is x y z:" << x_max << " " << y_max << " " << z_max << "\n";

  for (int x=x_min; x<x_max; x++) {
    for (int y=y_min; y<y_max; y++) {
      for (int z=z_min; z<z_max; z++) {
        Vector3 end_vec = rot.transform(Vector3((float) x*step_value+1.25f, (float) y*step_value, (float) z*step_value-0.5f));
        point3d endpoint (end_vec.x(), end_vec.y(), end_vec.z());
        tree.updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  // Truck's whole base
  x_min = - (int)(2.5f/resolution_value);
  x_max = - x_min;
  y_min = - (int)(1.5f/resolution_value);
  y_max = - y_min;
  z_min = - (int)(1.5f/resolution_value);
  z_max = - z_min;

  cout << "Base's size is x y z:" << x_max << " " << y_max << " " << z_max << "\n";

  for (int x=x_min; x<x_max; x++) {
    for (int y=y_min; y<y_max; y++) {
      for (int z=z_min; z<z_max; z++) {
        Vector3 end_vec = rot.transform(Vector3((float) x*0.05f+0.5f, (float) y*0.05f, (float) z*0.05f+0.75f));
        point3d endpoint (end_vec.x(), end_vec.y(), end_vec.z());
        tree.updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  // insert some measurements of free cells

  //Truck's region above landing area
  x_min = - (int)(1.5f/resolution_value);
  x_max = - x_min;
  y_min = - (int)(1.5f/resolution_value);
  y_max = - y_min;
  z_min = - (int)(1.0f/resolution_value);
  z_max = - z_min;
  for (int x=x_min; x<x_max; x++) {
    for (int y=y_min; y<y_max; y++) {
      for (int z=z_min; z<z_max; z++) {
        Vector3 end_vec = rot.transform(Vector3((float) x*0.05f, (float) y*0.05f, (float) z*0.05f-0.5f));
        point3d endpoint (end_vec.x(), end_vec.y(), end_vec.z());
        tree.updateNode(endpoint, false);  // integrate 'free' measurement
      }
    }
  }

  cout << "Above's size is x y z:" << x_max << " " << y_max << " " << z_max << "\n";

  cout << endl;
  cout << "performing some queries:" << endl;
  
  point3d query (0., 0., 0.);
  OcTreeNode* result = tree.search (query);
  print_query_info(query, result);

  query = point3d(-1.,-1.,-1.);
  result = tree.search (query);
  print_query_info(query, result);

  query = point3d(1.,1.,1.);
  result = tree.search (query);
  print_query_info(query, result);


  cout << endl;
  tree.writeBinary("truck0.1.bt");
  cout << "wrote example file simple_tree.bt" << endl << endl;
  cout << "now you can use octovis to visualize: octovis simple_tree.bt"  << endl;
  cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;  

}
