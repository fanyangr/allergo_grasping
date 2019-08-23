//CDatabaseRedisClient.cpp

#include "CDatabaseRedisClient.hpp"
#include <json/json.h>
#include <sstream>

// hacked version of eigenToStringArrayJson to prevent duplicate symbols till we have a better solution
template<typename Derived>
void hEigentoStringArrayJSON(const Eigen::MatrixBase<Derived>& x, std::string& arg_str)
{
	std::stringstream ss;
	bool row_major = true;
	if(x.cols() == 1) row_major = false; //This is a Vector!
	arg_str = "[";
	if(row_major)
	{// [1 2 3; 4 5 6] == [ [1, 2, 3], [4, 5, 6] ]
	  for(int i=0;i<x.rows();++i){
	    if(x.rows() > 1){
	      // If it is only one row, don't need the second one
	      if(i>0) arg_str.append(",[");
	      else arg_str.append("[");
	    }
	    else if(i>0) arg_str.append(",");
	    for(int j=0;j<x.cols();++j){
	      ss<<x(i,j);
	      if(j>0) arg_str.append(",");
	      arg_str.append(ss.str());
	      ss.str(std::string());
	    }
	    if(x.rows() > 1){
	      // If it is only one row, don't need the second one
	      arg_str.append("]");
	    }
	  }
	  arg_str.append("]");
	}
	else
	{// [1 2 3; 4 5 6] == 1 4 2 5 3 6
	  for(int j=0;j<x.cols();++j){
	    if(x.cols() > 1){
	      // If it is only one row, don't need the second one
	      if(j>0) arg_str.append(",[");
	      else arg_str.append("[");
	    }
	    else if(j>0) arg_str.append(",");
	    for(int i=0;i<x.rows();++i){
	      ss<<x(i,j);
	      if(i>0) arg_str.append(",");
	      arg_str.append(ss.str());
	      ss.str(std::string());
	    }
	    if(x.cols() > 1){
	      // If it is only one row, don't need the second one
	      arg_str.append("]");
	    }
	  }
	  arg_str.append("]");
	}
}

template<typename Derived>
bool hEigenFromStringArrayJSON(Eigen::MatrixBase<Derived>& x, const std::string &arg_str)
{
	Json::Value jval;
    Json::Reader json_reader;
    if(!json_reader.parse(arg_str,jval))
    { return false; }

	if(!jval.isArray()) return false; //Must be an array..
	unsigned int nrows = jval.size();
	if(nrows < 1) return false; //Must have elements.

	bool is_matrix = jval[0].isArray();
	if(!is_matrix)
	{
	  x.setIdentity(nrows,1);//Convert it into a vector.
	  for(int i=0;i<nrows;++i) x(i,0) = jval[i].asDouble();
	}
	else
	{
	  unsigned int ncols = jval[0].size();
	  if(ncols < 1) return false; //Must have elements.
	  for(int i=0;i<nrows;++i){
	    if(ncols != jval[i].size()) return false;
	    for(int j=0;j<ncols;++j)
	      x(i,j) = jval[i][j].asDouble();
	  }
	}
	return true;
}

// write raw eigen vector: TODO: use typename template for MatrixBase<Derived>
void CDatabaseRedisClient::setEigenMatrixDerived(const string &cmd_mssg, const Eigen::VectorXd &set_vector) {
	string data_mssg;
	// serialize
	hEigentoStringArrayJSON(set_vector, data_mssg); //this never fails
	// set to server
	setCommandIs(cmd_mssg, data_mssg);
}

// read raw eigen vector: TODO: use typename template for MatrixBase<Derived>
void CDatabaseRedisClient::getEigenMatrixDerived(const string &cmd_mssg, Eigen::VectorXd &ret_vector) {
	bool success = getCommandIs(cmd_mssg);
	// deserialize
  	if(success && !hEigenFromStringArrayJSON(ret_vector, reply_->str)) {
		throw(runtime_error("Could not deserialize json to eigen data!"));
	}
	freeReplyObject((void*)reply_);	
}

// write raw eigen vector: TODO: use typename template for MatrixBase<Derived>
void CDatabaseRedisClient::setEigenMatrixDerived(const string &cmd_mssg, const Eigen::Vector2d &set_vector) {
        string data_mssg;
        // serialize
        hEigentoStringArrayJSON(set_vector, data_mssg); //this never fails
        // set to server
        setCommandIs(cmd_mssg, data_mssg);
}

// write raw eigen vector: TODO: use typename template for MatrixBase<Derived>
void CDatabaseRedisClient::setEigenMatrixDerived(const string &cmd_mssg, const Eigen::Vector3d &set_vector) {
        string data_mssg;
        // serialize
        hEigentoStringArrayJSON(set_vector, data_mssg); //this never fails
        // set to server
        setCommandIs(cmd_mssg, data_mssg);
}

// write raw eigen vector: TODO: use typename template for MatrixBase<Derived>
void CDatabaseRedisClient::setEigenMatrixDerived(const string &cmd_mssg, const Eigen::Vector4d &set_vector) {
        string data_mssg;
        // serialize
        hEigentoStringArrayJSON(set_vector, data_mssg); //this never fails
        // set to server
        setCommandIs(cmd_mssg, data_mssg);
}

// read raw eigen vector: TODO: use typename template for MatrixBase<Derived>
void CDatabaseRedisClient::getEigenMatrixDerived(const string &cmd_mssg, Eigen::Vector2d &ret_vector) {
        bool success = getCommandIs(cmd_mssg);
        // deserialize
        if(success && !hEigenFromStringArrayJSON(ret_vector, reply_->str)) {
                throw(runtime_error("Could not deserialize json to eigen data!"));
        }
        freeReplyObject((void*)reply_);
}

// read raw eigen vector: TODO: use typename template for MatrixBase<Derived>
void CDatabaseRedisClient::getEigenMatrixDerived(const string &cmd_mssg, Eigen::Vector3d &ret_vector) {
        bool success = getCommandIs(cmd_mssg);
        // deserialize
        if(success && !hEigenFromStringArrayJSON(ret_vector, reply_->str)) {
                throw(runtime_error("Could not deserialize json to eigen data!"));
        }
        freeReplyObject((void*)reply_);
}

// read raw eigen vector: TODO: use typename template for MatrixBase<Derived>
void CDatabaseRedisClient::getEigenMatrixDerived(const string &cmd_mssg, Eigen::Vector4d &ret_vector) {
        bool success = getCommandIs(cmd_mssg);
        // deserialize
        if(success && !hEigenFromStringArrayJSON(ret_vector, reply_->str)) {
                throw(runtime_error("Could not deserialize json to eigen data!"));
        }
        freeReplyObject((void*)reply_);
}

