/*
 *    Copyright (C) 2021 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <limits>
#include <list>
#include <string>
#include <type_traits>
#include <vector>
#include <Eigen/Geometry>

/* Utility function to print Matrix */
template<template <typename, typename...> class Container,
                   typename T,
                   typename... Args>
//disable for string, which is std::basic_string<char>, a container itself
typename std::enable_if<!std::is_convertible<Container<T, Args...>, std::string>::value &&
                        !std::is_constructible<Container<T, Args...>, std::string>::value,
                            std::ostream&>::type
operator<<(std::ostream& os, const Container<T, Args...>& con)
{
    os << " ";
    for (auto& elem: con)
        os << elem << " ";

    os << "\n";
    return os;
}

// Values to determine exists in the world 
int max_lambda_value = 50;
int min_lambda_value = -50;
int hits_to_reach_top_thr = 50;
int min_insert_dist_thr = 1000;
float top_thr = 0.7;
float bot_thr = 0.3;
auto s = -hits_to_reach_top_thr/(log(1/top_thr-1));
auto integrator = [](auto x){return 1/(1 + exp(-x/s));};

cv::RNG rng(12345);
cv::Scalar color_1 = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256));
cv::Scalar color_2 = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256));

int person_name_idx = 0;

// Values for drawing pictures
int y_res = 640;
int x_res = 480;
float y_res_f = 640.0;
float x_res_f = 480.0;
float zero = 0;

template<typename T>
void handle_negatives(std::vector<std::vector<T>>& matrix, 
                      bool allowed = true)
{
    T minval = std::numeric_limits<T>::max();
    
    for (auto& elem: matrix)
        for (auto& num: elem)
            minval = std::min(minval, num);
        
    if (minval < 0) {
        if (!allowed) { //throw
            throw std::runtime_error("Only non-negative values allowed");
        }
        else { // add abs(minval) to every element to create one zero
            minval = abs(minval);
            
            for (auto& elem: matrix)
                for (auto& num: elem)
                    num += minval;
        }
    }
}

/* Ensure that the matrix is square by the addition of dummy rows/columns if necessary */
template<typename T>
void pad_matrix(std::vector<std::vector<T>>& matrix)
{
    std::size_t i_size = matrix.size();
    std::size_t j_size = matrix[0].size();
    
    if (i_size > j_size) {
        for (auto& vec: matrix)
            vec.resize(i_size, std::numeric_limits<T>::max());
    }
    else if (i_size < j_size) {
        while (matrix.size() < j_size)
            matrix.push_back(std::vector<T>(j_size, std::numeric_limits<T>::max()));
    }
}

/* For each row of the matrix, find the smallest element and subtract it from every 
 * element in its row.  
 * For each col of the matrix, find the smallest element and subtract it from every 
 * element in its col. Go to Step 2. */
template<typename T>
void step1(std::vector<std::vector<T>>& matrix, 
           int& step)
{
    // process rows
    for (auto& row: matrix) {
        auto smallest = *std::min_element(begin(row), end(row));
        if (smallest > 0)        
            for (auto& n: row)
                n -= smallest;
    }
    
    // process cols
    int sz = matrix.size(); // square matrix is granted
    for (int j=0; j<sz; ++j) {
        T minval = std::numeric_limits<T>::max();
        for (int i=0; i<sz; ++i) {
            minval = std::min(minval, matrix[i][j]);
        }
        
        if (minval > 0) {
            for (int i=0; i<sz; ++i) {
                matrix[i][j] -= minval;
            }
        }
    }
   
    step = 2;
}

/* helper to clear the temporary vectors */
inline void clear_covers(std::vector<int>& cover) 
{
    for (auto& n: cover) n = 0;
}

/* Find a zero (Z) in the resulting matrix.  If there is no starred zero in its row or 
 * column, star Z. Repeat for each element in the matrix. Go to Step 3.  In this step, 
 * we introduce the mask matrix M, which in the same dimensions as the cost matrix and 
 * is used to star and prime zeros of the cost matrix.  If M(i,j)=1 then C(i,j) is a 
 * starred zero,  If M(i,j)=2 then C(i,j) is a primed zero.  We also define two vectors 
 * RowCover and ColCover that are used to "cover" the rows and columns of the cost matrix.
 * In the nested loop (over indices i and j) we check to see if C(i,j) is a zero value 
 * and if its column or row is not already covered.  If not then we star this zero 
 * (i.e. set M(i,j)=1) and cover its row and column (i.e. set R_cov(i)=1 and C_cov(j)=1).
 * Before we go on to Step 3, we uncover all rows and columns so that we can use the 
 * cover vectors to help us count the number of starred zeros. */
template<typename T>
void step2(const std::vector<std::vector<T>>& matrix, 
           std::vector<std::vector<int>>& M, 
           std::vector<int>& RowCover,
           std::vector<int>& ColCover, 
           int& step)
{
    int sz = matrix.size();
    
    for (int r=0; r<sz; ++r) 
        for (int c=0; c<sz; ++c) 
            if (matrix[r][c] == 0)
                if (RowCover[r] == 0 && ColCover[c] == 0) {
                    M[r][c] = 1;
                    RowCover[r] = 1;
                    ColCover[c] = 1;
                }
            
    clear_covers(RowCover); // reset vectors for posterior using
    clear_covers(ColCover);
    
    step = 3;
}


/* Cover each column containing a starred zero.  If K columns are covered, the starred 
 * zeros describe a complete set of unique assignments.  In this case, Go to DONE, 
 * otherwise, Go to Step 4. Once we have searched the entire cost matrix, we count the 
 * number of independent zeros found.  If we have found (and starred) K independent zeros 
 * then we are done.  If not we procede to Step 4.*/
void step3(const std::vector<std::vector<int>>& M, 
           std::vector<int>& ColCover,
           int& step)
{
    int sz = M.size();
    int colcount = 0;
    
    for (int r=0; r<sz; ++r)
        for (int c=0; c<sz; ++c)
            if (M[r][c] == 1)
                ColCover[c] = 1;
            
    for (auto& n: ColCover)
        if (n == 1)
            colcount++;
    
    if (colcount >= sz) {
        step = 7; // solution found
    }
    else {
        step = 4;
    }
}

// Following functions to support step 4
template<typename T>
void find_a_zero(int& row, 
                 int& col,
                 const std::vector<std::vector<T>>& matrix,
                 const std::vector<int>& RowCover,
                 const std::vector<int>& ColCover)
{
    int r = 0;
    int c = 0;
    int sz = matrix.size();
    bool done = false;
    row = -1;
    col = -1;
    
    while (!done) {
        c = 0;
        while (true) {
            if (matrix[r][c] == 0 && RowCover[r] == 0 && ColCover[c] == 0) {
                row = r;
                col = c;
                done = true;
            }
            c += 1;
            if (c >= sz || done)
                break;
        }
        r += 1;
        if (r >= sz)
            done = true;
    }
}

bool star_in_row(int row, 
                 const std::vector<std::vector<int>>& M)
{
    bool tmp = false;
    for (unsigned c = 0; c < M.size(); c++)
        if (M[row][c] == 1)
            tmp = true;
    
    return tmp;
}


void find_star_in_row(int row,
                      int& col, 
                      const std::vector<std::vector<int>>& M)
{
    col = -1;
    for (unsigned c = 0; c < M.size(); c++)
        if (M[row][c] == 1)
            col = c;
}


/* Find a noncovered zero and prime it.  If there is no starred zero in the row containing
 * this primed zero, Go to Step 5.  Otherwise, cover this row and uncover the column 
 * containing the starred zero. Continue in this manner until there are no uncovered zeros
 * left. Save the smallest uncovered value and Go to Step 6. */
template<typename T>
void step4(const std::vector<std::vector<T>>& matrix, 
           std::vector<std::vector<int>>& M, 
           std::vector<int>& RowCover,
           std::vector<int>& ColCover,
           int& path_row_0,
           int& path_col_0,
           int& step)
{
    int row = -1;
    int col = -1;
    bool done = false;

    while (!done){
        find_a_zero(row, col, matrix, RowCover, ColCover);
        
        if (row == -1){
            done = true;
            step = 6;
        }
        else {
            M[row][col] = 2;
            if (star_in_row(row, M)) {
                find_star_in_row(row, col, M);
                RowCover[row] = 1;
                ColCover[col] = 0;
            }
            else {
                done = true;
                step = 5;
                path_row_0 = row;
                path_col_0 = col;
            }
        }
    }
}

// Following functions to support step 5
void find_star_in_col(int c, 
                      int& r,
                      const std::vector<std::vector<int>>& M)
{
    r = -1;
    for (unsigned i = 0; i < M.size(); i++)
        if (M[i][c] == 1)
            r = i;
}

void find_prime_in_row(int r, 
                       int& c, 
                       const std::vector<std::vector<int>>& M)
{
    for (unsigned j = 0; j < M.size(); j++)
        if (M[r][j] == 2)
            c = j;
}

void augment_path(std::vector<std::vector<int>>& path, 
                  int path_count, 
                  std::vector<std::vector<int>>& M)
{
    for (int p = 0; p < path_count; p++)
        if (M[path[p][0]][path[p][1]] == 1)
            M[path[p][0]][path[p][1]] = 0;
        else
            M[path[p][0]][path[p][1]] = 1;
}

void erase_primes(std::vector<std::vector<int>>& M)
{
    for (auto& row: M)
        for (auto& val: row)
            if (val == 2)
                val = 0;
}


/* Construct a series of alternating primed and starred zeros as follows.  
 * Let Z0 represent the uncovered primed zero found in Step 4.  Let Z1 denote the 
 * starred zero in the column of Z0 (if any). Let Z2 denote the primed zero in the 
 * row of Z1 (there will always be one).  Continue until the series terminates at a 
 * primed zero that has no starred zero in its column.  Unstar each starred zero of 
 * the series, star each primed zero of the series, erase all primes and uncover every 
 * line in the matrix.  Return to Step 3.  You may notice that Step 5 seems vaguely 
 * familiar.  It is a verbal description of the augmenting path algorithm (for solving
 * the maximal matching problem). */
void step5(std::vector<std::vector<int>>& path, 
           int path_row_0, 
           int path_col_0, 
           std::vector<std::vector<int>>& M, 
           std::vector<int>& RowCover,
           std::vector<int>& ColCover,
           int& step)
{
    int r = -1;
    int c = -1;
    int path_count = 1;
    
    path[path_count - 1][0] = path_row_0;
    path[path_count - 1][1] = path_col_0;
    
    bool done = false;
    while (!done) {
        find_star_in_col(path[path_count - 1][1], r, M);
        if (r > -1) {
            path_count += 1;
            path[path_count - 1][0] = r;
            path[path_count - 1][1] = path[path_count - 2][1];
        }
        else {done = true;}
        
        if (!done) {
            find_prime_in_row(path[path_count - 1][0], c, M);
            path_count += 1;
            path[path_count - 1][0] = path[path_count - 2][0];
            path[path_count - 1][1] = c;
        }
    }
    
    augment_path(path, path_count, M);
    clear_covers(RowCover);
    clear_covers(ColCover);
    erase_primes(M);
    
    step = 3;
}

// methods to support step 6
template<typename T>
void find_smallest(T& minval, 
                   const std::vector<std::vector<T>>& matrix, 
                   const std::vector<int>& RowCover,
                   const std::vector<int>& ColCover)
{
    for (unsigned r = 0; r < matrix.size(); r++)
        for (unsigned c = 0; c < matrix.size(); c++)
            if (RowCover[r] == 0 && ColCover[c] == 0)
                if (minval > matrix[r][c])
                    minval = matrix[r][c];
}

/* Add the value found in Step 4 to every element of each covered row, and subtract it 
 * from every element of each uncovered column.  Return to Step 4 without altering any
 * stars, primes, or covered lines. Notice that this step uses the smallest uncovered 
 * value in the cost matrix to modify the matrix.  Even though this step refers to the
 * value being found in Step 4 it is more convenient to wait until you reach Step 6 
 * before searching for this value.  It may seem that since the values in the cost 
 * matrix are being altered, we would lose sight of the original problem.  
 * However, we are only changing certain values that have already been tested and 
 * found not to be elements of the minimal assignment.  Also we are only changing the 
 * values by an amount equal to the smallest value in the cost matrix, so we will not
 * jump over the optimal (i.e. minimal assignment) with this change. */
template<typename T>
void step6(std::vector<std::vector<T>>& matrix, 
           const std::vector<int>& RowCover,
           const std::vector<int>& ColCover,
           int& step)
{
    T minval = std::numeric_limits<T>::max();
    find_smallest(minval, matrix, RowCover, ColCover);
    
    int sz = matrix.size();
    for (int r = 0; r < sz; r++)
        for (int c = 0; c < sz; c++) {
            if (RowCover[r] == 1)
                matrix[r][c] += minval;
            if (ColCover[c] == 0)
                matrix[r][c] -= minval;
    }
    
    step = 4;
}

/* Calculates the optimal cost from mask matrix */
template<template <typename, typename...> class Container,
         typename T,
         typename... Args>
T output_solution(const Container<Container<T,Args...>>& original,
                  const std::vector<std::vector<int>>& M)
{
    T res = 0;
    
    for (unsigned j=0; j<original.begin()->size(); ++j)
        for (unsigned i=0; i<original.size(); ++i)
            if (M[i][j]) {
                auto it1 = original.begin();
                std::advance(it1, i);
                auto it2 = it1->begin();
                std::advance(it2, j);
                res += *it2;
                continue;                
            }
            
    return res;
}


/* Main function of the algorithm */
template<template <typename, typename...> class Container,
         typename T,
         typename... Args>
std::vector<std::vector<int>> hungarian(const Container<Container<T,Args...>>& original,
          bool allow_negatives = true)
{  
    /* Initialize data structures */
    
    // Work on a vector copy to preserve original matrix
    // Didn't passed by value cause needed to access both
    std::vector<std::vector<T>> matrix (original.size(), 
                                        std::vector<T>(original.begin()->size()));
    
    auto it = original.begin();
    for (auto& vec: matrix) {         
        std::copy(it->begin(), it->end(), vec.begin());
        it = std::next(it);
    }
    
    // handle negative values -> pass true if allowed or false otherwise
    // if it is an unsigned type just skip this step
    if (!std::is_unsigned<T>::value) {
        handle_negatives(matrix, allow_negatives);
    }
    
    
    // make square matrix
    pad_matrix(matrix);
    std::size_t sz = matrix.size();
    
    /* The masked matrix M.  If M(i,j)=1 then C(i,j) is a starred zero,  
     * If M(i,j)=2 then C(i,j) is a primed zero. */
    std::vector<std::vector<int>> M (sz, std::vector<int>(sz, 0));
    
    /* We also define two vectors RowCover and ColCover that are used to "cover" 
     *the rows and columns of the cost matrix C*/
    std::vector<int> RowCover (sz, 0);
    std::vector<int> ColCover (sz, 0);
    
    int path_row_0, path_col_0; //temporary to hold the smallest uncovered value
    
    // Array for the augmenting path algorithm
    std::vector<std::vector<int>> path (sz+1, std::vector<int>(2, 0));
    
    /* Now Work The Steps */
    bool done = false;
    int step = 1;
    while (!done) {
        switch (step) {
            case 1:
                step1(matrix, step);
                break;
            case 2:
                step2(matrix, M, RowCover, ColCover, step);
                break;
            case 3:
                step3(M, ColCover, step);
                break;
            case 4:
                step4(matrix, M, RowCover, ColCover, path_row_0, path_col_0, step);
                break;
            case 5:
                step5(path, path_row_0, path_col_0, M, RowCover, ColCover, step);
                break;
            case 6:
                step6(matrix, RowCover, ColCover, step);
                break;
            case 7:
                for (auto& vec: M) {vec.resize(original.begin()->size());}
                M.resize(original.size());
                done = true;
                break;
            default:
                done = true;
                break;
        }
    }

    return M;
}

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
    QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	G->write_to_json_file("./"+agent_name+".json");
	G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    agent_name = params["agent_name"].value;
	agent_id = stoi(params["agent_id"].value);

	tree_view = params["tree_view"].value == "true";
	graph_view = params["graph_view"].value == "true";
	qscene_2d_view = params["2d_view"].value == "true";
	osg_3d_view = params["3d_view"].value == "true";

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
		// create graph
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, ""); // Init nodes
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;

        inner_eigen = G->get_inner_eigen_api();

		//dsr update signals
//		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
//		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
//		connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_attrs_slot);
//		connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
//		connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

		// Graph viewer
		using opts = DSR::DSRViewer::view;
		int current_opts = 0;
		opts main = opts::none;
		if(tree_view)
		{
		    current_opts = current_opts | opts::tree;
		}
		if(graph_view)
		{
		    current_opts = current_opts | opts::graph;
		    main = opts::graph;
		}
		if(qscene_2d_view)
		{
		    current_opts = current_opts | opts::scene;
		}
		if(osg_3d_view)
		{
		    current_opts = current_opts | opts::osg;
		}
		graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));
		this->Period = period;
		timer.start(Period);
	}

}

std::optional<vector<cv::Point3f>> SpecificWorker::get_person_coords(RoboCompHumanCameraBody::Person p)
{
    vector<cv::Point3f> points;
    RoboCompHumanCameraBody::TJoints person_tjoints = p.joints;
    list<RoboCompHumanCameraBody::KeyPoint> huesitos;


    for(auto item : person_tjoints)
    {
        std::string key = item.first;
        if (item.second.x != 0 && item.second.y != 0 && item.second.z != 0 && item.second.i != 0 && item.second.j != 0)
        {
            huesitos.push_back(item.second);
        }
    }

//    std::cout << "NUMERO DE JOINTS: " << huesitos.size() << "" << "PERSON ID: " << p.id << endl;
    if(huesitos.size() < 6)
    {
        return {};
    }

    else
    {
        float avg_x_robot = 0, avg_y_robot = 0,avg_z_robot=0, avg_x_world = 0, avg_y_world = 0,avg_z_world=0;
        Eigen::Vector3f trans_vect_1(0, -0.06, -0.12);
        Eigen::Vector3f trans_vect_2(0, -0.04, -1.55);
        for(auto kp: huesitos)
        {
            Eigen::Vector3f joint_pos(kp.x, kp.z, kp.y);
            Eigen::AngleAxisf z_axis_rotation_matrix (servo_position, Eigen::Vector3f::UnitZ());
            joint_pos = z_axis_rotation_matrix * joint_pos;
            joint_pos = joint_pos + trans_vect_1;
            Eigen::AngleAxisf x_axis_rotation_matrix (0.314159, Eigen::Vector3f::UnitX());
            joint_pos = x_axis_rotation_matrix * joint_pos;
            Eigen::Vector3f joint_pos_robot = joint_pos + trans_vect_2;
            auto joint_pos_robot_cast = joint_pos_robot.cast <double>()*1000;
            // For world
            if(auto joint_pos_world_double = inner_eigen->transform("world", joint_pos_robot_cast, "robot"); joint_pos_world_double.has_value())
            {
                auto joint_pos_world = joint_pos_world_double.value().cast <float>();
                avg_x_robot += joint_pos_robot.x();
                avg_y_robot += joint_pos_robot.y();
                avg_z_robot += joint_pos_robot.z();

                avg_x_world += joint_pos_world.x();
                avg_y_world += joint_pos_world.y();
                avg_z_world += joint_pos_world.z();
            }
            else return {};
        }
        avg_x_robot = avg_x_robot/huesitos.size();
        avg_y_robot = avg_y_robot/huesitos.size();
        avg_z_robot = avg_z_robot/huesitos.size();

        avg_x_world = avg_x_world/huesitos.size();
        avg_y_world = avg_y_world/huesitos.size();
        avg_z_world = avg_z_world/huesitos.size();


        cv::Point3f point_robot(avg_x_robot*1000,avg_y_robot*1000,avg_z_robot*1000);
        cv::Point3f point_world(avg_x_world,avg_y_world,avg_z_world);

        points.push_back(point_robot);
        points.push_back(point_world);

        return points;
    }
}

cv::Point2i SpecificWorker::get_person_pixels(RoboCompHumanCameraBody::Person p)
{
    RoboCompHumanCameraBody::TJoints person_tjoints = p.joints;
    list<RoboCompHumanCameraBody::KeyPoint> huesitos;
    for(auto item : person_tjoints)
    {
        std::string key = item.first;
        if (item.second.x != 0 && item.second.y != 0 && item.second.z != 0 && item.second.i != 0 && item.second.j != 0)
        {
            huesitos.push_back(item.second);
        }
    }

    if(huesitos.empty())
    {
        auto kp = person_tjoints.begin()->second;
        cv::Point2i point(kp.i,kp.j);
        return point;
    }
    else
    {
        float avg_i = 0, avg_j = 0;

        for(auto kp: huesitos)
        {
            avg_i += kp.i;
            avg_j += kp.j;
        }
        avg_i = avg_i/huesitos.size();
        avg_j = avg_j/huesitos.size();

//         cout << "MEAN i pos: " << avg_i << endl;
//         cout << "MEAN j pos: " << avg_j << endl;

        cv::Point2i point(avg_i, avg_j);
        return point;
    }
}

void SpecificWorker::compute()
{
    // Creating white image with dimension 480x640
    cv::Mat black_picture = cv::Mat::zeros(y_res, x_res, CV_8UC3);

    auto servo_data = this->jointmotorsimple_proxy->getMotorState("");
    servo_position = servo_data.pos;
    // Taking people data through proxy
    RoboCompHumanCameraBody::PeopleData people_data = this->humancamerabody_proxy->newPeopleData();

    RoboCompHumanCameraBody::People people_list = people_data.peoplelist;
    // Generating camera image
    // RoboCompCameraRGBDSimple::TImage image = this->camerargbdsimple_proxy->getImage("123456789");
    
    // cv::Mat frame(cv::Size(image.height, image.width), CV_8UC3, &image.image[0], cv::Mat::AUTO_STEP);

    // Vector to include people data with ne new struct (world position and orientation added)
    vector<SpecificWorker::PersonData> person_data_vector;
    // Iterating each person
    for(int i=0;i<people_list.size();i++)
    {
        RoboCompHumanCameraBody::Person person = people_list[i];
        // cout << "Person ID: " << person.id << endl;
        RoboCompHumanCameraBody::TJoints person_tjoints = person.joints;
        std::vector<cv::Point> pixel_vector;
        // Iterating TJoints
        for(auto item : person_tjoints)
        {
            // Appending joint pixels to a vector
            cv::Point pixel;
            pixel.x = item.second.i;
            pixel.y = item.second.j;
            pixel_vector.push_back(pixel);
        }
        // Creating person with the new stucture
        SpecificWorker::PersonData person_data;
        person_data.id = person.id;
        person_data.joints = person.joints;
        if(auto coords = get_person_coords(person); coords.has_value())
        {
            cv::Point2i person_pixels = get_person_pixels(person);
            person_data.personCoords_robot = coords.value()[0];
            person_data.personCoords_world = coords.value()[1];
            person_data.pixels = person_pixels;
            person_data.orientation = calculate_orientation(person);

            // Insert person in person data vector
            person_data_vector.push_back(person_data);

            // Represent image
            cv::Rect person_box = cv::boundingRect(pixel_vector);
            cv::rectangle(black_picture, cv::Rect(person_box.x, person_box.y, person_box.width, person_box.height), color_1, 2);
            cv::Point square_center;
            square_center.x = person_box.x+(person_box.width/2);
            square_center.y = person_box.y+(person_box.height/2);
            cv::circle(black_picture, square_center,12,color_2);
            for(int k=0;k<pixel_vector.size();k++)
            {
                cv::circle(black_picture, pixel_vector[k],12,color_1);
            }
        }
        else continue;
    }
    update_graph(person_data_vector);
    cv::imshow("RGB image", black_picture);
    cv::waitKey(1);
}

std::int32_t SpecificWorker::increase_lambda_cont(std::int32_t lambda_cont)
// Increases lambda_cont in 1, to the maximum value and returns the new value. Returns the new lambda_cont value.
{
    std::int32_t nlc = lambda_cont + 1;
    if(nlc < max_lambda_value) {return nlc;}
    else {return max_lambda_value;}
}

std::int32_t SpecificWorker::decrease_lambda_cont(std::int32_t lambda_cont)
// Decreases lambda_cont in 1, to the minimun value and returns the new value. Returns the new lambda_cont value.
{
    std::int32_t nlc = lambda_cont - 1;
    if(nlc > min_lambda_value) {return nlc;}
    else {return min_lambda_value;}
}

float SpecificWorker::distance_3d(cv::Point3f p1, cv::Point3f p2)
{
//    cout << "p1: " << p1 << endl;
//    cout << "p2: " << p2 << endl;
//    cout << "DISTANCIA: " << cv::norm(p1-p2) << endl;
    return cv::norm(p1-p2);
}

cv::Point3f SpecificWorker::dictionary_values_to_3d_point(RoboCompHumanCameraBody::KeyPoint item)
{
    cv::Point3f point;
    float x = item.x;
    float y = item.y;
    float z = item.z;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

cv::Point3f SpecificWorker::cross_product(cv::Point3f p1, cv::Point3f p2)
{
    cv::Point3f point;
    point.x = p1.y * p2.z - p1.z * p2.y;
    point.y = p1.z * p2.x - p1.x * p2.z;
    point.z = p1.x * p2.y - p1.y * p2.x;
    return point;
}

//function to calculate dot product of two vectors
float SpecificWorker::dot_product3D(cv::Point3f vector_a, cv::Point3f vector_b) {
    float product = 0;

    product = product + vector_a.x * vector_b.x;
    product = product + vector_a.y * vector_b.y;
    product = product + vector_a.z * vector_b.z;

    return product;

}

float SpecificWorker::dot_product(cv::Point2f vector_a, cv::Point2f vector_b) {
    float product = 0;

    product = product + vector_a.x * vector_b.x;
    product = product + vector_a.y * vector_b.y;

    return product;

}

float SpecificWorker::get_degrees_between_vectors(cv::Point2f vector_1, cv::Point2f vector_2, std::string format)
{
    // Returns the angle between two vectors in the 2d plane (v2 respect v1)

    if (format.compare("radians") == 0 && format.compare("degrees") == 0)
    {
        cout << "Invalid angle format. Format parameter should be \"radians\" or \"degrees\"" << endl;
        return 0.0;
    }

    // Getting unitary vectors
    cv::Point2f u_vector_1 = vector_1/cv::norm(vector_1);
    // cout << "u_vector_1: ("<< u_vector_1.x << ","<< u_vector_1.y << ")" << endl;
    cv::Point2f u_vector_2 = vector_2/cv::norm(vector_2);
    // cout << "vector_2: ("<< vector_2.x << ","<< vector_2.y << ")" << endl;
    // cout << "NORM: "<< cv::norm(vector_2) << endl;
    // cout << "u_vector_2: ("<< u_vector_2.x << ","<< u_vector_2.y << ")" << endl;

    // Extra vector: u_vector_2 rotated /90 degrees
    cv::Point2f u_vector_2_90;
    u_vector_2_90.x = cos(-M_PI / 2) * u_vector_2.x - sin(-M_PI / 2) * u_vector_2.y;
    u_vector_2_90.y = sin(-M_PI / 2) * u_vector_2.x + cos(-M_PI / 2) * u_vector_2.y;

    // cout << "u_vector_2_90: ("<< u_vector_2_90.x << ","<< u_vector_2_90.y << ")" << endl;
    // Dot product of u_vector_1 with u_vector_2 and u_vector_2_90
    //float dp = u_vector_1.x * u_vector_2.x + u_vector_1.y * u_vector_2.y;
    //float dp_90 = u_vector_1.x * u_vector_2_90.x + u_vector_1.y * u_vector_2_90.y;

    // Dot product of u_vector_1 with u_vector_2 and urobot.value_vector_2_90
    float dp = dot_product(u_vector_1, u_vector_2);
    // cout << "DP: " << dp << endl;
    float dp_90 = dot_product(u_vector_1, u_vector_2_90);
    // cout << "DP_90: " << dp_90 << endl;

    // Comprobating if the angle is over 180 degrees and adapting
    float ret;
    if(dp_90 < 0){ret = M_PI + (M_PI-acos(dp));}
    else{ret = acos(dp);}

    // cout << "RET: " << ret << endl;

    // Returning value
    if (format.compare("radians") == 0) {return ret;}
    else {return (ret*180/M_PI);}
}

float SpecificWorker::calculate_orientation(RoboCompHumanCameraBody::Person person)
{
    RoboCompHumanCameraBody::TJoints person_tjoints = person.joints;
    bool left_found= false, base_found= false, right_found = false;
    cv::Point3f base_p, right_p, left_p;

    for(auto item : person_tjoints)
    {
        std::string key = item.first;

        // Base point

        if (base_found == false && (key.compare("17")==0 || key.compare("6")==0 || key.compare("5")==0 || key.compare("2")==0 || key.compare("1")==0))
        {
            base_found = true;
            base_p = dictionary_values_to_3d_point(item.second);
            // cout << "KEYPOINT BASEP: "<< key <<endl;
        }

        // Right point

        if (right_found == false && (key.compare("12")==0 || key.compare("4")==0))
        {
            right_found = true;
            right_p = dictionary_values_to_3d_point(item.second);
            // cout << "KEYPOINT RIGHTP: "<< key <<endl;
        }

        // Left point

        if (left_found == false && (key.compare("11")==0 || key.compare("3")==0))
        {
            left_found = true;
            left_p = dictionary_values_to_3d_point(item.second);
            // cout << "KEYPOINT LEFTP: "<< key <<endl;
        }

        if(base_found == true && right_found == true && left_found == true)
        {
            //break;
        }

        // cout << "CLAVE: " << key << ", VALOR: (" << item.second.x << "," <<item.second.y << "," <<item.second.z << ")" << endl;
    }

    if(base_found == false || right_found == false || left_found == false)
    {
        // cout << "Points not found. Can't calculate orientation." << endl;
        return 0.0;
    }

    // Considering "clavícula" as coordinate center. Passing leg points to "clavícula" reference system

    cv::Point3f left_v = left_p - base_p;
    cv::Point3f right_v = right_p - base_p;

    // Calculating perpendicular vector

    cv::Point3f normal = cross_product(left_v, right_v);
    cv::Point2f vector_1, vector_2;
    vector_1.x = 0;
    vector_1.y = 1;
    vector_2.x = normal.x;
    vector_2.y = normal.z;

    // cout << "vector_2: (" << vector_2.x << ","<<vector_2.y << ")" << endl;

    float angle = get_degrees_between_vectors(vector_1, vector_2, "radians");
    // cout << "Ángulo: " << angle << endl;
    return angle;
}

void SpecificWorker::remove_person(DSR::Node person_node, bool direct_remove)
{
    float score = 0;
    if(direct_remove == true){int score = 0;}
    else
    {
        if(auto person_lc = G->get_attrib_by_name<lambda_cont_att>(person_node); person_lc.has_value())
        {
            if(auto person_id = G->get_attrib_by_name<person_id_att>(person_node); person_id.has_value())
            {
                cout << "calculating lambda value for person " << person_id.value() << endl;
                int nlc = decrease_lambda_cont(person_lc.value());
                cout << "//////////////// NLC decreasing:   " << nlc << endl;
                G->add_or_modify_attrib_local<lambda_cont_att>(person_node, nlc);
                G->update_node(person_node);
                score = integrator(nlc);
            }
        }
    }
    if((score <= bot_thr) or (direct_remove == true))
    {
        auto people_space_nodes = G->get_nodes_by_type("personal_space");
        auto mind_nodes = G->get_nodes_by_type("transform");
        auto parent_id = person_node.id();

        if(auto robot_node = G->get_node("robot"); robot_node.has_value())
        {
            // Checks if a person dissapears while has a following or talking edge, to avoid delete it
            if(auto following_edge = G->get_edge(robot_node.value().id(), person_node.id(), "following"); following_edge.has_value())
            {
                ;
            }
            else
            {
                // Getting personal space nodes
                for(int i=0; i<people_space_nodes.size();i++)
                {
                    auto act_space_node = people_space_nodes[i];
                    if(auto act_space_node_person_id = G->get_attrib_by_name<person_id_att>(act_space_node); act_space_node_person_id.has_value())
                    {
                        if(auto person_id = G->get_attrib_by_name<person_id_att>(person_node); person_id.has_value())
                        {
                            if(act_space_node_person_id.value() == person_id.value());
                            {
                                people_space_nodes.erase(people_space_nodes.begin()+i);
                                G->delete_node(act_space_node.id());
                                break;
                            }
                        }
                    }
                }

                for(int i=0; i<mind_nodes.size();i++)
                {
                    auto act_mind_node = mind_nodes[i];
                    if(auto act_mind_node_id = G->get_attrib_by_name<person_id_att>(act_mind_node); act_mind_node_id.has_value())
                    {
                        if(auto person_id = G->get_attrib_by_name<person_id_att>(person_node); person_id.has_value())
                        {
                            if(act_mind_node_id.value() == person_id.value())
                            {
                                // cout << "parent_id: " << parent_id << endl;
                                // cout << "mind id: " << act_mind_node_id << endl;
                                mind_nodes.erase(mind_nodes.begin()+i);
                                G->delete_node(act_mind_node.id());
                                G->delete_edge(person_node.id(), act_mind_node.id(),"has");
                                break;
                            }
                        }
                    }
                }
                if(auto world_node = G->get_node("world"); world_node.has_value())
                {
                    G->delete_edge(world_node.value().id(), person_node.id(),"RT");
                    G->delete_node(person_node.id());
                }
            }
        }
    }
}

void SpecificWorker::update_person(DSR::Node node, cv::Point3f world_coords, cv::Point3f robot_coords, float orientation, cv::Point2i pixels)
{
    float score = 0;
    if(auto world_node = G->get_node("world"); world_node.has_value())
    {
        auto node_value = world_node.value();
        if(auto robot_node = G->get_node("robot"); robot_node.has_value())
        {
            // Check if person was lost
            if(auto is_lost = G->get_attrib_by_name<lost_att>(node); is_lost.has_value())
            {
                if(is_lost.value() == true)
                {
                    G->add_or_modify_attrib_local<lost_att>(node, false);
                }

                // Modify distance from human to robot
                float dist_to_robot = sqrt(pow(robot_coords.x,2) + pow(robot_coords.y,2));

                G->add_or_modify_attrib_local<distance_to_robot_att>(node, dist_to_robot);
                G->add_or_modify_attrib_local<pixel_x_att>(node, pixels.x);
                G->add_or_modify_attrib_local<pixel_y_att>(node, pixels.y);
                G->update_node(node);

                std::vector<float> new_position_vector_robot = {robot_coords.x, robot_coords.y, robot_coords.z};
                std::vector<float> new_position_vector_world = {world_coords.x, world_coords.y, world_coords.z};
                std::vector<float> orientation_vector = {0.0, orientation, 0.0};
                try
                {
                    if(auto edge_world = rt->get_edge_RT(node_value, node.id()); edge_world.has_value())
                    {
                        if(auto last_pos = G->get_attrib_by_name<rt_translation_att>(edge_world.value()); last_pos.has_value())
                        {
                            std::cout << "POS x: " << last_pos.value().get()[0] << " POS y: " << last_pos.value().get()[1] << endl;
                            G->modify_attrib_local<rt_rotation_euler_xyz_att>(edge_world.value(), orientation_vector);
                            G->modify_attrib_local<rt_translation_att>(edge_world.value(), new_position_vector_world);

                            if (G->insert_or_assign_edge(edge_world.value()))
                            {
                                std::cout << __FUNCTION__ << " Edge successfully modified: " << robot_node.value().id() << "->" << node.id()
                                          << " type: RT" << std::endl;
                            }
                            else
                            {
                                std::cout << __FUNCTION__ << ": Fatal error modifying new edge: " << robot_node.value().id() << "->" << node.id()
                                          << " type: RT" << std::endl;
                                std::terminate();
                            }

                            // Lambda_cont increment
                            if(auto person_lc = G->get_attrib_by_name<lambda_cont_att>(node); person_lc.has_value())
                            {
                                int nlc = increase_lambda_cont(person_lc.value());
                                G->add_or_modify_attrib_local<lambda_cont_att>(node, nlc);
                                G->update_node(node);

                                // Taking values to comparate
                                if(auto person_distance = G->get_attrib_by_name<distance_to_robot_att>(node); person_distance.has_value())
                                {
                                    score = integrator(nlc);



                                    // Check if person if ready. If it is, a ready edge is created

                                    if(score >= top_thr && person_distance.value() < 1500 && nlc > 0 && (orientation > (2*M_PI - (M_PI/6)) || orientation < (M_PI/6)))
                                    {
                                        DSR::Edge edge = DSR::Edge::create<ready_edge_type>(robot_node.value().id(), node.id());
                                        if (G->insert_or_assign_edge(edge))
                                        {
                                            std::cout << __FUNCTION__ << " Edge successfully inserted: " << robot_node.value().id() << "->" << node.id()
                                                      << " type: ready" << std::endl;
                                        }
                                        else
                                        {
                                            std::cout << __FUNCTION__ << ": Fatal error inserting new edge: " << robot_node.value().id() << "->" << node.id()
                                                      << " type: ready" << std::endl;
                                            std::terminate();
                                        }
                                    }
                                    else
                                    {
                                        if(auto ready_edge = G->get_edge(robot_node.value().id(), node.id(), "ready"); ready_edge.has_value())
                                        {
                                            G->delete_edge(robot_node.value().id(), node.id(),"ready");
                                        }
                                    }
                                    G->update_node(node);
                                }
                            }
                        }
                    }
                }
                catch(int e)
                {
                    cout << e << endl;
                }
            }
        }
    }
}

void SpecificWorker::insert_mind(std::uint64_t parent_id, std::int32_t person_id)
{
    if( auto parent_node = G->get_node(parent_id); parent_node.has_value())
    {
        float pos_x = rand()%(400-250 + 1) + 250;
        float pos_y = rand()%(170-(-30) + 1) + (-30);
        std::string person_id_str = std::to_string(person_id);
        std::string node_name = "person_mind_" + person_id_str;
        DSR::Node new_node = DSR::Node::create<transform_node_type>(node_name);
        G->add_or_modify_attrib_local<person_id_att>(new_node, person_id);
        G->add_or_modify_attrib_local<parent_att>(new_node, parent_id);
        G->add_or_modify_attrib_local<pos_x_att>(new_node, pos_x);
        G->add_or_modify_attrib_local<pos_y_att>(new_node, pos_y);
        try
        {
            G->insert_node(new_node);
            DSR::Edge edge = DSR::Edge::create<has_edge_type>(parent_node.value().id(), new_node.id());
            if (G->insert_or_assign_edge(edge))
            {
                std::cout << __FUNCTION__ << " Edge successfully inserted: " << parent_node.value().id() << "->" << new_node.id()
                          << " type: has" << std::endl;
            }
            else
            {
                std::cout << __FUNCTION__ << ": Fatal error inserting new edge: " << parent_node.value().id() << "->" << new_node.id()
                          << " type: has" << std::endl;
                std::terminate();
            }


        }
        catch(int e)
        {
            cout << "Problema" << endl;
        }
    }
}

void SpecificWorker::insert_person(cv::Point3f world_coords, cv::Point3f robot_coords,  float orientation, bool direct_insert, cv::Point2i pixels)
{
    if(auto robot_node = G->get_node("robot"); robot_node.has_value())
    {
        if(auto world_node = G->get_node("world"); robot_node.has_value())
        {
            float pos_x = rand()%(120-(-100) + 1) + (-100);
            float pos_y = rand()%(-100-(-370) + 1) + (-370);
            int id = person_name_idx;
            person_name_idx += 1;

            std::string person_id_str = std::to_string(id);
            std::string node_name = "person_" + person_id_str;

            DSR::Node new_node = DSR::Node::create<person_node_type>(node_name);
            G->add_or_modify_attrib_local<person_id_att>(new_node, id);
            G->add_or_modify_attrib_local<followed_att>(new_node, false);
            G->add_or_modify_attrib_local<lost_att>(new_node, false);
            G->add_or_modify_attrib_local<pixel_x_att>(new_node, pixels.x);
            G->add_or_modify_attrib_local<pixel_y_att>(new_node, pixels.y);
            G->add_or_modify_attrib_local<is_ready_att>(new_node, direct_insert);

            int lc = 0;
            if(direct_insert == true){lc = hits_to_reach_top_thr;}
            else{lc = 1;}

            G->add_or_modify_attrib_local<parent_att>(new_node, world_node.value().id());
            G->add_or_modify_attrib_local<lambda_cont_att>(new_node, lc);
            G->add_or_modify_attrib_local<distance_to_robot_att>(new_node, robot_coords.y);
            G->add_or_modify_attrib_local<pos_x_att>(new_node, pos_x);
            G->add_or_modify_attrib_local<pos_y_att>(new_node, pos_y);
            try
            {
                auto id_result = G->insert_node(new_node);
                DSR::Edge edge_world = DSR::Edge::create<RT_edge_type>(world_node.value().id(), new_node.id());
                std::vector<float> new_position_vector_world = {world_coords.x, world_coords.y, world_coords.z};
                std::vector<float> orientation_vector = {0.0, orientation, 0.0};
                G->add_attrib_local<rt_rotation_euler_xyz_att>(edge_world, orientation_vector);
                G->add_attrib_local<rt_translation_att>(edge_world, new_position_vector_world);
                if (G->insert_or_assign_edge(edge_world))
                {
                    std::cout << __FUNCTION__ << " Edge successfully inserted: " << world_node.value().id() << "->" << new_node.id()
                              << " type: RT" << std::endl;
                }
                else
                {
                    std::cout << __FUNCTION__ << ": Fatal error inserting new edge: " << world_node.value().id() << "->" << new_node.id()
                              << " type: RT" << std::endl;
                    std::terminate();
                }

                insert_mind(id_result.value(), id);
            }
            catch(int e)
            {
                cout << "Problema" << endl;
            }
        }
    }
}

void SpecificWorker::update_graph(vector<SpecificWorker::PersonData> people_list)
{
    if(auto world_node = G->get_node("world"); world_node.has_value())
    {
        auto people_nodes = G->get_nodes_by_type("person");

        // If there's no people in image and nodes, just don't do anything
        if (people_nodes.size() == 0 && people_list.size() == 0)
        {
            return;
        }

        // If there's people in image but not in nodes, append them
        if(people_nodes.size() == 0 && people_list.size() != 0)
        {
            for(auto p: people_list)
            {
                insert_person(p.personCoords_world ,p.personCoords_robot, p.orientation, true, p.pixels);
            }
        }

        // Calculating to see if some person has to be erased
        else
        {
            // Rows for nodes, columns for people
            vector<vector<int>> comparisons;
            vector<int> matched_nodes;
            vector<int> matched_people;
            if(people_list.size() > 0)
            {
                std::cout << "CASE 3 1" << endl;
                for(auto p: people_nodes)
                {
                    std::cout << "CASE 3 2" << endl;
                    int row = 0;
                    vector<int> people_distance_to_nodes;
                    for(int i=0;i<people_list.size();i++)
                    {
                        if(auto edge = G->get_edge(world_node.value().id(), p.id(), "RT"); edge.has_value())
                        {

                            if(auto g_coords = G->get_attrib_by_name<rt_translation_att>(edge.value()); g_coords.has_value())
                            {
                                cv::Point3f g_coords_point;
                                g_coords_point.x = g_coords.value().get()[0]; g_coords_point.y = g_coords.value().get()[1]; g_coords_point.z = g_coords.value().get()[2];
                                auto p_c = people_list[i].personCoords_world;
                                auto diff_dist = distance_3d(g_coords_point, p_c);
                                people_distance_to_nodes.push_back(round(diff_dist));
                            }
                        }
                    }
                    row += 1;
                    comparisons.push_back(people_distance_to_nodes);
                }

                auto matches = hungarian(comparisons);

                for(int i = 0;i<matches.size();i++)
                {
                    for(int j = 0;j<matches[i].size();j++)
                    {
                        if(matches[i][j] == 1 && comparisons[i][j] < 1000)
                        {
                            std::cout << "FILA: " << i << " " << "COLUMNA: " << j << endl;

                            update_person(people_nodes[i], people_list[j].personCoords_world, people_list[j].personCoords_robot, people_list[j].orientation, people_list[j].pixels);
                            matched_nodes.push_back(i);
                            matched_people.push_back(j);
                        }

                    }
                }
            }

            // If list of people nodes is bigger than list of people, some nodes must be proposed to be deleted
            if(people_nodes.size() > people_list.size())
            {
                // If there's no people
                if(people_list.size() == 0)
                {
                    std::cout << "2" << endl;
                    for(int i = 0; i < people_nodes.size() ;i++)
                    {
                        // See if followed person is lost
                        if( auto follow_attr = G->get_attrib_by_name<followed_att>(people_nodes[i]); follow_attr.has_value())
                        {
                            std::cout << "2" << endl;
                            if(auto lost_attr = G->get_attrib_by_name<lost_att>(people_nodes[i]); lost_attr.has_value())
                            {
                                // Si la persona está siendo seguida, no aparece en la imagen, pero todavía no se ha perdido, se modifica a true el atributo de perdido
                                if(follow_attr.value() == true && lost_attr.value() == false)
                                {
                                    std::cout << "PERSONA PERDIDA" << endl;
                                    G->add_or_modify_attrib_local<lost_att>(people_nodes[i], true);
                                    G->update_node(people_nodes[i]);
                                }
                                    // Si la persona no aparece en la imagen, está siendo seguida y se ha perdido, se mantiene a la espera
                                else if(follow_attr.value() == true && lost_attr.value() == true)
                                {
                                    std::cout << "PERSONA TODAVIA PERDIDA" << endl;
                                }
                                    // Si no es el caso, se elimina
                                else remove_person(people_nodes[i], false);
                            }
                        }
                    }
                    return;
                }
                for(int i = 0; i < people_nodes.size() ;i++)
                {
                    if (not (std::find(matched_nodes.begin(), matched_nodes.end(), i) != matched_nodes.end()))
                    {
                        remove_person(people_nodes[i], false);
                    }
                }
            }

                // If list of people size is bigger than list of people nodes, new people must be inserted into the graph
            else if(people_nodes.size() < people_list.size())
            {
                for(int i = 0; i < people_nodes.size() ;i++)
                {
                    if (not (std::find(matched_people.begin(), matched_people.end(), i) != matched_people.end()))
                    {
                        insert_person(people_list[i].personCoords_world, people_list[i].personCoords_robot, people_list[i].orientation, false, people_list[i].pixels);
                    }
                }
            }
        }
    }

}

// void del_node_slot(std::uint64_t from)
// {
//     auto mind_nodes = G->get_nodes_by_type("transform");
//     auto person_nodes = G->get_nodes_by_type("person");
//     for(auto g: mind_nodes)
//     {
//         if(auto node_parent = G->get_attrib_by_name<parent_att>(g).value(); not has_value())
//         {
//             G->delete_node(g.id());
//         }
//     }
//     for(auto g: person_nodes)
//     {
//         if(auto node_parent = G->get_attrib_by_name<parent_att>(g).value(); not has_value())
//         {
//             G->delete_node(g.id());
//         }
//     }
// }

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

/**************************************/
// From the RoboCompCameraRGBDSimple you can call this methods:
// this->camerargbdsimple_proxy->getAll(...)
// this->camerargbdsimple_proxy->getDepth(...)
// this->camerargbdsimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompHumanCameraBody you can call this methods:
// this->humancamerabody_proxy->newPeopleData(...)

/**************************************/
// From the RoboCompHumanCameraBody you can use this types:
// RoboCompHumanCameraBody::TImage
// RoboCompHumanCameraBody::TGroundTruth
// RoboCompHumanCameraBody::KeyPoint
// RoboCompHumanCameraBody::Person
// RoboCompHumanCameraBody::PeopleData

