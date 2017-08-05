#ifndef _RRT_
#define _RRT_

#include <iostream>
#include <bits/stdc++.h>
#include "utils.hpp"

namespace rrt
{
	template <class T>
	class RRT
	{
	private:
		double halfDimensionX;
		double halfDimensionY;
		Utils::Point<T> origin;
		Utils::Point<T> startPoint;
		Utils::Point<T> endPoint;
		double stepLength;
		std::vector<Utils::Point<T> > pathPoints;
		int maxIterations;
		std::vector< std::pair< Utils::Point<T>, Utils::Point<T> > > tree; 
		unsigned int biasParameter;
		
	public:
		
		RRT(){
			srand(time(NULL));
		};
		RRT(Utils::Point<T> start,Utils::Point<T> end)
		{
			srand(time(NULL));			
			startPoint=start;
			endPoint=end;
		}
		
		virtual bool plan();
		virtual std::vector<Utils::Point<T> > getPointsOnPath();

		virtual void setEndPoints(Utils::Point<T> start, Utils::Point<T> end);
		virtual void setCheckPointFunction(bool (*f)(Utils::Point<T>));
		virtual void setStepLength(double value);
		virtual void setOrigin(Utils::Point<T> origin);
		virtual void setHalfDimensions(double x,double y);
		virtual void setBiasParameter(unsigned int);
		virtual void setMaxIterations(int);
	private:
		bool (*userCheck)(Utils::Point<T>);
		bool checkPoint(Utils::Point<T> pt);
		Utils::Point<T> generatePoint();
		Utils::Point<T> generateBiasedPoint();
		void growTree(Utils::Point<T>);
		Utils::Point<T> findClosestNode(Utils::Point<T>);
		Utils::Point<T> getParent(Utils::Point<T>);
		bool treeComplete();
		void generatePath(Utils::Point<T> first,Utils::Point<T> last);
		double dist(Utils::Point<T> a,Utils::Point<T> b);
	};
}

#endif