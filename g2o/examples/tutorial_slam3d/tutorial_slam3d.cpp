// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <cmath>

#include "simulator.h"
/*
#include "vertex_se2.h"
#include "vertex_point_xy.h"
#include "edge_se2.h"
#include "edge_se2_pointxy.h"
#include "types_tutorial_slam2d.h"
 */
#include "vertex_se3.h"
#include "vertex_pointxyz.h"
#include "edge_se3.h"
#include "se3quat.h"
#include "edge_se3_pointxyz.h"
//#include "types_tutorial_slam3d.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

//#include "hello.h"

using namespace std;
using namespace g2o;
using namespace g2o::tutorial;

//typedef Eigen::vector3d Vector3d;
const size_t N = 100;

int main()
{
	/*
	//  hello();

	init_tutorial_slam2d_types();

	// TODO simulate different sensor offset
	// simulate a robot observing landmarks while travelling on a grid
	SE2 sensorOffsetTransf(0.2, 0.1, -0.1);
	int numNodes = 300;
	Simulator simulator;
	simulator.simulate(numNodes, sensorOffsetTransf);
	 */
	// *********************************************************************************
	// * creating the optimization problem
	// *******************************************************************************

	typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
	typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

	// allocating the optimizer
	SparseOptimizer optimizer;
	SlamLinearSolver* linearSolver = new SlamLinearSolver();
	linearSolver->setBlockOrdering(false);
	SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
	OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(blockSolver);

	optimizer.setAlgorithm(solver);


	//  SE3 sensorOffsetTransf(0.2, 0.1, -0.1, 0.1, 0.1, 0.1);
	// add the parameter representing the sensor offset
	//  ParameterSE3Offset* sensorOffset = new ParameterSE3Offset;
	//  sensorOffset->setOffset(sensorOffsetTransf);
	//  sensorOffset->setId(0);
	//  optimizer.addParameter(sensorOffset);

	for(size_t i=0;i<N;i++){
		Eigen::Quaterniond q(0.0, 0.0, 0.0, 1.0);
		Eigen::Vector3d t;
		t << (double)i+0.01*rand()/(RAND_MAX + 1.0), 0.01*rand()/(RAND_MAX + 1.0), 0.01*rand()/(RAND_MAX + 1.0);
		cout << "t_" << i << " = " << t.transpose() << endl;
		SE3Quat p(q,t);
		VertexSE3* robot =  new VertexSE3;
		robot->setId(i);
		robot->setEstimate(p);
		optimizer.addVertex(robot);

	}
	for(size_t i=0;i<N-1;i++){	//相対座標を定義する。
		EdgeSE3* odom = new EdgeSE3;
		if(i==N-2)
		odom->vertices()[0] = optimizer.vertex(0);
		else
		odom->vertices()[0] = optimizer.vertex(i+1);
		odom->vertices()[1] = optimizer.vertex(i);
		Eigen::Isometry3d test;
		/*	test << 1.0, 0.0, 0.0, 1.0,
			0.0, 1.0, 0.0, 0.0,
			0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 1.0;
		 */
		for(int i=0;i<4;i++){
			for(int j=0;j<4;j++){
				if(i==j)
					test(i,j) = 1.0;
				else if((i==0)&&(j==3))
					test(i,j) = 1.0;
				else
					test(i,j) = 0.0;
				cout << test(i,j) << " ";
			}
			cout << endl;
		}
		if(i==N-2){
			test(0,3)=-(double)N;
			test(1,3)=-3.0;
		}
		odom->setMeasurement(test);
		Eigen::Matrix<double, 6, 6> it;
		it << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		   0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
		   0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
		   0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		   0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
		   0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
		   odom->setInformation(it);
		optimizer.addEdge(odom);
	}
	/*
	// adding the odometry to the optimizer
	// first adding all the vertices
	cerr << "Optimization: Adding robot poses ... ";
	for (size_t i = 0; i < simulator.poses().size(); ++i) {
	const Simulator::GridPose& p = simulator.poses()[i];
	const SE2& t = p.simulatorPose; 
	VertexSE2* robot =  new VertexSE2;
	robot->setId(p.id);
	robot->setEstimate(t);
	optimizer.addVertex(robot);
	}
	cerr << "done." << endl;

	// second add the odometry constraints
	cerr << "Optimization: Adding odometry measurements ... ";
	for (size_t i = 0; i < simulator.odometry().size(); ++i) {
	const Simulator::GridEdge& simEdge = simulator.odometry()[i];

	EdgeSE2* odometry = new EdgeSE2;
	odometry->vertices()[0] = optimizer.vertex(simEdge.from);
	odometry->vertices()[1] = optimizer.vertex(simEdge.to);
	odometry->setMeasurement(simEdge.simulatorTransf);
	odometry->setInformation(simEdge.information);
	optimizer.addEdge(odometry);
	}
	cerr << "done." << endl;

	// add the landmark observations
	cerr << "Optimization: add landmark vertices ... ";
	for (size_t i = 0; i < simulator.landmarks().size(); ++i) {
	const Simulator::Landmark& l = simulator.landmarks()[i];
	VertexPointXY* landmark = new VertexPointXY;
	landmark->setId(l.id);
	landmark->setEstimate(l.simulatedPose);
	optimizer.addVertex(landmark);
	}
	cerr << "done." << endl;
	 */
	/*
	   cerr << "Optimization: add landmark observations ... ";
	   for (size_t i = 0; i < simulator.landmarkObservations().size(); ++i) {
	   const Simulator::LandmarkEdge& simEdge = simulator.landmarkObservations()[i];
	   EdgeSE2PointXY* landmarkObservation =  new EdgeSE2PointXY;
	   landmarkObservation->vertices()[0] = optimizer.vertex(simEdge.from);
	   landmarkObservation->vertices()[1] = optimizer.vertex(simEdge.to);
	   landmarkObservation->setMeasurement(simEdge.simulatorMeas);
	   landmarkObservation->setInformation(simEdge.information);
	   landmarkObservation->setParameterId(0, sensorOffset->id());
	   optimizer.addEdge(landmarkObservation);
	   }
	   cerr << "done." << endl;
	 */

	// *********************************************************************************
	// * optimization
	// ********************************************************************************

	// dump initial state to the disk
	optimizer.save("tutorial_before.g2o");

	// prepare and run the optimization
	// fix the first robot pose to account for gauge freedom
	VertexSE3* firstRobotPose = dynamic_cast<VertexSE3*>(optimizer.vertex(0));
	firstRobotPose->setFixed(true);
	optimizer.setVerbose(true);

	cerr << "Optimizing" << endl;
	optimizer.initializeOptimization();
	optimizer.optimize(10);
	cerr << "done." << endl;

	optimizer.save("tutorial_after.g2o");

	// freeing the graph memory
	optimizer.clear();

	// destroy all the singletons
	Factory::destroy();
	OptimizationAlgorithmFactory::destroy();
	HyperGraphActionLibrary::destroy();

	return 0;
}
