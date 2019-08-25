#ifndef CDATABASEREDISCLIENT_H
#define CDATABASEREDISCLIENT_H

#include <Eigen/Core>
#include <hiredis/hiredis.h>
#include <string>
#include <iostream>
#include <stdexcept>
using std::cout;
using std::string;
using std::runtime_error;

struct HiredisServerInfo {
public:
    char *hostname_;
    int port_;
    timeval timeout_;
};

class CDatabaseRedisClient {
public:
	CDatabaseRedisClient ()
		: context_(NULL),
		reply_(NULL)
	{// do nothing
	}

	~CDatabaseRedisClient () {
		redisFree(context_);
		freeReplyObject((void*)reply_);
	}

	// init with server info
	void serverIs(HiredisServerInfo server) {
		// delete existing connection
		if (NULL != context_ || NULL == server.hostname_) {
			redisFree(context_);
		}
		if (NULL == server.hostname_) {
			// nothing to do
			return;
		}
		// set new server info
		server_info_ = server;
		// connect to new server
		redisContext* tmp_context = redisConnectWithTimeout(server.hostname_, server.port_, server.timeout_);
        if (NULL == tmp_context) {
        	throw(runtime_error("Could not allocate redis context."));
        }
    	if (tmp_context->err) {
			std::string err = string("Could not connect to redis server : ") + string(tmp_context->errstr);
			redisFree(tmp_context);
			throw(runtime_error(err.c_str()));
        }
    	// set context, ping server
    	context_ = tmp_context;
	}

public:
	// set expiry (ms) on an existing db key
	void keyExpiryIs(const string& key, const uint expiry_ms) {
		reply_ = (redisReply *)redisCommand(context_, "PEXPIRE %s %s", key.c_str(), std::to_string(expiry_ms).c_str());
		// NOTE: write commands dont check for write errors.
		freeReplyObject((void*)reply_);
	}

	bool getCommandIs(const string &cmd_mssg) {
		reply_ = (redisReply *)redisCommand(context_, "GET %s", cmd_mssg.c_str());
		if (NULL == reply_ || REDIS_REPLY_ERROR == reply_->type) {
			throw(runtime_error("Server error in fetching data!"));
			//TODO: indicate what error
		}
		if (REDIS_REPLY_NIL == reply_->type) {
			// cout << "\nNo data on server.. Missing key?";
			return false;
		}
		return true;
	}

	bool setCommandIs(const string &cmd_mssg, const string &data_mssg) {
		reply_ = (redisReply *)redisCommand(context_, "SET %s %s", cmd_mssg.c_str(), data_mssg.c_str());
		// NOTE: set commands dont check for write errors.
      	freeReplyObject((void*)reply_);
      	return true;
	}

	// write raw eigen vector: TODO: use typename template for MatrixBase<Derived>
	void setEigenMatrixDerived(const string &cmd_mssg, const Eigen::VectorXd &set_vector);

	// write raw eigen vector: TODO: use typename template for MatrixBase<Derived>
        void setEigenMatrixDerived(const string &cmd_mssg, const Eigen::Vector2d &set_vector);

	// write raw eigen vector: TODO: use typename template for MatrixBase<Derived>
        void setEigenMatrixDerived(const string &cmd_mssg, const Eigen::Vector3d &set_vector);

	// write raw eigen vector: TODO: use typename template for MatrixBase<Derived>
        void setEigenMatrixDerived(const string &cmd_mssg, const Eigen::Vector4d &set_vector);

	// read raw eigen vector: TODO: use typename template for MatrixBase<Derived>
	void getEigenMatrixDerived(const string &cmd_mssg, Eigen::VectorXd &ret_vector);

	// read raw eigen vector: TODO: use typename template for MatrixBase<Derived>
        void getEigenMatrixDerived(const string &cmd_mssg, Eigen::Vector2d &ret_vector);

	// read raw eigen vector: TODO: use typename template for MatrixBase<Derived>
        void getEigenMatrixDerived(const string &cmd_mssg, Eigen::Vector3d &ret_vector);

	// read raw eigen vector: TODO: use typename template for MatrixBase<Derived>
        void getEigenMatrixDerived(const string &cmd_mssg, Eigen::Vector4d &ret_vector);

public: // server connectivity tools
	void ping() {
		// PING server to make sure things are working..
        reply_ = (redisReply *)redisCommand(context_,"PING");
        cout<<"\n\nDriver Redis Task : Pinged Redis server. Reply is, "<<reply_->str<<"\n";
        freeReplyObject((void*)reply_);
	}

public:
	HiredisServerInfo server_info_;
	redisContext *context_;
    redisReply *reply_;
	//TODO: decide if needed. Currently, we throw up if server disconnects
	//bool connected_;
private:
	// internal function. prepends robot name to command
	static inline string robotCmd(string robot_name, string cmd) {
		return robot_name + ":" + cmd;
	}
};

#endif //CDATABASEREDISCLIENT_H

