#ifndef MATTEST_H
#define MATTEST_H
#define EXIT_SUCCESS 0
//#define foreach BOOST_FOREACH
#define EIGEN_DEFAULT_TO_ROW_MAJOR
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <iostream>
#include <queue>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include "getopt.h"
#include <matio_private.h>
#include <boost/numeric/ublas/io.hpp>
#include <boost/variant.hpp>
#include <boost/foreach.hpp>
using namespace std;
using namespace boost::numeric;
using namespace Eigen;
namespace MatIO {

	static void Mat_Message(const char*) {}
	static void Mat_Message(const char*,const char*) {}
	static void Mat_Message(const char*,int) {}
	static void Mat_Message(const char*,size_t) {}
	static void Mat_Message(const char*,float) {}

	/*
	* struct that adds on top of the current matvar_t implementation,
	* it adds children so that it can be traversed, and it can take either
	* a matrix aor string
	*/
	struct matvarplus_t {
		matvar_t* matvar; // current one
		matvarplus_t* parent;
		vector<matvarplus_t*> children;
		boost::variant<Eigen::MatrixXf*, std::string > mat_char; //
		matvarplus_t(matvar_t* matvar, matvarplus_t* parent):matvar(matvar),parent(parent) {}
		~matvarplus_t() {
			free(matvar); 
			delete parent;
			if (mat_char.type() == typeid(Eigen::MatrixXf*))
				delete boost::get< Eigen::MatrixXf *>(mat_char);
		}
	};
	/*
	struct result_package {
	boost::variant<Eigen::MatrixXf, vector< string > > data;

	};*/
	static vector<string>* get_charlist(matvarplus_t* root, const char* arguments ) {
		char buf[2048];
		memcpy(buf, arguments, strlen(arguments)+1);
		vector<char*> entries;
		char* temp;
		int curr_index = 0;
		temp = strtok(buf, "->");
		while (temp != NULL) {
			entries.push_back(temp);
			temp=strtok(NULL, "->");
		}
		std::priority_queue<matvarplus_t*> q;
		q.push(root);
		if (strcmp(entries[0], q.top()->matvar->name)==0) {
			while (!q.empty()&& curr_index < entries.size()) {
				matvarplus_t* temp = q.top();
				curr_index++;
				q.pop();
				Mat_Message("Parent Name: %s\n", temp->matvar->name);
				if (temp->matvar->class_type == MAT_C_CELL) {
					if (temp->children.size() > 0) {
						vector<string>* t = new vector<string>();
						// checking if children is all type of Char
						bool all_chars = true;
						for (int j = 0; j < temp->children.size(); j++) {
							if (temp->children[j]->matvar->class_type != MAT_C_CHAR) {
								all_chars = false;
								delete t;
								break;
							}
							t->push_back(boost::get<string>(temp->children[j]->mat_char));
						}
						if (all_chars)
							return t;

					}
				} else 
					Mat_Message("Name: %s",temp->matvar->name);
				BOOST_FOREACH (matvarplus_t* j, temp->children) {
					if (strcmp(j->matvar->name,entries[curr_index]) == 0)
						q.push(j); 
				}
			}
		}
		return NULL;
		/*	vector<char*> entries;
		va_list param;
		va_start(param, argc);
		for (int i = 0; i < argc; ++i) {
		entries.push_back(va_arg

		}
		va_end(param);
		*/
	}

	static bool get_matrix(MatrixXf& res, matvarplus_t* root, const char* arguments ) {
		//Mat_Message("Before Parsing");
		char buf[2048];
		memcpy(buf, arguments, strlen(arguments)+1);
		vector<char*> entries;
		char* temp;
		int curr_index = 0;
		temp = strtok(buf, "->");
		//Mat_Message("After first strtok");
		while (temp != NULL) {
			entries.push_back(temp);
			temp=strtok(NULL, "->");
		}
		//Mat_Message("Done Parsing!");
		std::priority_queue<matvarplus_t*> q;
		q.push(root);
		if (strcmp(entries[0], q.top()->matvar->name)==0) {
			while (!q.empty()&& curr_index < entries.size()) {

				//	Mat_Message("In while loop %s", entries[0]);
				matvarplus_t* temp = q.top();
				curr_index++;
				//	Mat_Message("My Name %s\n", temp->matvar->name);
				q.pop();
				/*if (temp->matvar->class_type == MAT_C_STRUCT) {
				if (temp->children.size() > 0) {
				vector<string>* t = new vector<string>();
				// checking if children is all type of Char
				bool all_chars = true;
				for (int j = 0; j < temp->children.size(); j++) {
				if (temp->children[j]->matvar->class_type != MAT_C_CHAR) {
				all_chars = false;
				break;
				}
				t->push_back(string(temp->children[j]->matvar->name));
				}
				if (all_chars)
				return t;
				}
				} else */
				if (temp->matvar->class_type == MAT_C_SINGLE || temp->matvar->class_type == MAT_C_DOUBLE) {
					res = *boost::get< Eigen::MatrixXf *>(temp->mat_char);
					//return new MatrixXf(*boost::get< Eigen::MatrixXf *>(temp->mat_char));
					return true;
				} else 
					BOOST_FOREACH (matvarplus_t* j, temp->children) {
						if (j->matvar->class_type != MAT_C_CHAR && strcmp(j->matvar->name, entries[curr_index])==0)//if (j->matvar->class_type == MAT_C_SINGLE || j->matvar->class_type == MAT_C_DOUBLE || j->matvar->class_type == MAT_C_STRUCT)
						{q.push(j);}
				}
			}
		}
		return false;
		/*	vector<char*> entries;
		va_list param;
		va_start(param, argc);
		for (int i = 0; i < argc; ++i) {
		entries.push_back(va_arg

		}
		va_end(param);
		*/
	}
	template <class T>
	static T get_value(matvarplus_t* root, const char* arguments) {
		MatrixXf temp;

		get_matrix(temp, root, arguments);
		return (T)(temp(0,0));

	}
	static void print_bfstree(matvarplus_t* root) {
		std::priority_queue<matvarplus_t*> q;
		q.push(root);
		while (!q.empty()) {
			matvarplus_t* temp = q.top();
			q.pop();
			if (temp->matvar->class_type == MAT_C_CHAR) {
				Mat_Message(boost::get< string > (temp->mat_char).c_str());
			} else if (temp->matvar->class_type == MAT_C_SINGLE || temp->matvar->class_type == MAT_C_DOUBLE) {
				//			Mat_Message("Name: %s\n", temp->matvar->name);
				Eigen::MatrixXf*& t = boost::get< Eigen::MatrixXf *>(temp->mat_char);
				for (int i = 0; i < t->rows() && i < 15; ++i) {
					for (int j = 0; j < t->cols() && j < 15; ++j) {
						Mat_Message("%f ", (*t)(i,j));
					}
					Mat_Message("\n");
				}
			} else 
				//			Mat_Message("Name: %s",temp->matvar->name);
				BOOST_FOREACH (matvarplus_t* j, temp->children) {
					q.push(j);
			}
		}
	}
	static float
		Mat_GetFloatNumber(enum matio_types type, void *data)
	{
		switch ( type ) {
		case MAT_T_DOUBLE:
			return static_cast<float>(*(double*) data);
			break;
		case MAT_T_SINGLE:
			return *(float*) data;
			break;
		}
	}
	static	void
		Mat_VarUBlas( matvarplus_t *matvarplus, int printdata )
	{
		int i, j;
		const char *class_type_desc[16] = {"Undefined","Cell Array","Structure",
			"Object","Character Array","Sparse Array","Double Precision Array",
			"Single Precision Array", "8-bit, signed integer array",
			"8-bit, unsigned integer array","16-bit, signed integer array",
			"16-bit, unsigned integer array","32-bit, signed integer array",
			"32-bit, unsigned integer array","64-bit, signed integer array",
			"64-bit, unsigned integer array"};
		const char *data_type_desc[23] = {"Unknown","8-bit, signed integer",
			"8-bit, unsigned integer","16-bit, signed integer",
			"16-bit, unsigned integer","32-bit, signed integer",
			"32-bit, unsigned integer","IEEE 754 single-precision","RESERVED",
			"IEEE 754 double-precision","RESERVED","RESERVED",
			"64-bit, signed integer","64-bit, unsigned integer", "Matlab Array",
			"Compressed Data","Unicode UTF-8 Encoded Character Data",
			"Unicode UTF-16 Encoded Character Data",
			"Unicode UTF-32 Encoded Character Data","","String","Cell Array",
			"Structure"};

		if ( matvarplus->matvar == NULL )
			return;
		if ( matvarplus->matvar->name )
			Mat_Message("      Name: %s\n", matvarplus->matvar->name);
		Mat_Message("      Rank: %d\n", matvarplus->matvar->rank);
		if ( matvarplus->matvar->rank == 0 )
			return;
		Mat_Message("Dimensions: %d",matvarplus->matvar->dims[0]);
		// check for more dimensions
		for ( i = 1; i < matvarplus->matvar->rank; i++ )
			Mat_Message(" x %d",matvarplus->matvar->dims[i]);
		Mat_Message("\n");
		Mat_Message("Class Type: %s",class_type_desc[matvarplus->matvar->class_type]);
		if ( matvarplus->matvar->isComplex )
			Mat_Message(" (complex)");
		Mat_Message("\n");
		if ( matvarplus->matvar->data_type )
			Mat_Message(" Data Type: %s\n", data_type_desc[matvarplus->matvar->data_type]);

		if ( matvarplus->matvar->data == NULL || matvarplus->matvar->data_size < 1 ) {
			return;
		} else if ( MAT_C_STRUCT == matvarplus->matvar->class_type ) {
			matvar_t **fields = (matvar_t **)matvarplus->matvar->data;
			int nfields = matvarplus->matvar->nbytes / matvarplus->matvar->data_size;
			Mat_Message("Fields[%d] {\n", nfields);
			// Adding children
			for ( i = 0; i < nfields; i++ ) {
				matvarplus_t* temp = new matvarplus_t(fields[i],matvarplus);
				matvarplus->children.push_back(temp);
				Mat_VarUBlas(temp,printdata);
			}
			Mat_Message("}\n");
			return;
		} else if ( MAT_C_CELL == matvarplus->matvar->class_type ) {
			matvar_t **cells = (matvar_t **)matvarplus->matvar->data;
			int ncells = matvarplus->matvar->nbytes / matvarplus->matvar->data_size;
			Mat_Message("{\n");
			for ( i = 0; i < ncells; i++ ) {
				matvarplus_t* temp = new matvarplus_t(cells[i],matvarplus);
				matvarplus->children.push_back(temp);
				Mat_VarUBlas(temp,printdata);
			}
			Mat_Message("}\n");
			return;
		} else if ( !printdata ) {
			return;
		}

		Mat_Message("{\n");

		if ( matvarplus->matvar->rank > 2 ) {
			Mat_Message("I can't print more than 2 dimensions\n");
		} else if ( matvarplus->matvar->rank == 1 && matvarplus->matvar->dims[0] > 15 ) {
			Mat_Message("I won't print more than 15 elements in a vector\n");
		} else if ( matvarplus->matvar->rank==2 ) {
			switch( matvarplus->matvar->class_type ) {
			case MAT_C_DOUBLE:
			case MAT_C_SINGLE:
				{
					size_t stride = Mat_SizeOf(matvarplus->matvar->data_type);

					char *data = (char*) matvarplus->matvar->data;
					Eigen::MatrixXf*& mat_temp = boost::get< Eigen::MatrixXf *> (matvarplus->mat_char);
					(mat_temp) = new Eigen::MatrixXf(matvarplus->matvar->dims[0],matvarplus->matvar->dims[1]);
					for ( i = 0; i < matvarplus->matvar->dims[0]; i++ ) {
						for ( j = 0; j < matvarplus->matvar->dims[1]; j++ ) {
							size_t idx = matvarplus->matvar->dims[0]*j+i;
							float f = Mat_GetFloatNumber(matvarplus->matvar->data_type,
								data+idx*stride);
							(*mat_temp)(i,j) = f;	
						}
					}
					break;
				}
			case MAT_C_CHAR:
				{

					if ( !printdata )
						break;
					if ( matvarplus->matvar->dims[0] == 1 ) {
						matvarplus->mat_char = string((char*) matvarplus->matvar->data);
						Mat_Message( "===================================\n");
					} else {
						cout << "Char array in data field currently UNSUPPORTED" << endl;
						char *data = (char*)matvarplus->matvar->data;
						for ( i = 0; i < matvarplus->matvar->dims[0]; i++ ) {
							//j = 0;
							for ( j = 0; j < matvarplus->matvar->dims[1]; j++ ) {
								Mat_Message("%c",data[j*matvarplus->matvar->dims[0]+i]);
							}
							Mat_Message("\n");
						}
					}
					break;
				}
			} 
		}

		Mat_Message("}\n");

		return;
	}
	static int indent = 0;
	static int printdata = 1;
	static void
		print_default(matvarplus_t *matvarplus)
	{
		if ( NULL == matvarplus->matvar )
			return;

		switch ( matvarplus->matvar->class_type ) {
		case MAT_C_DOUBLE:
		case MAT_C_SINGLE:
		case MAT_C_INT64:
		case MAT_C_UINT64:
		case MAT_C_INT32:
		case MAT_C_UINT32:
		case MAT_C_INT16:
		case MAT_C_UINT16:
		case MAT_C_INT8:
		case MAT_C_UINT8:
		case MAT_C_CHAR:
		case MAT_C_SPARSE:
			Mat_VarUBlas(matvarplus, printdata ); // do mat_var print to print out all these types
			break;
		case MAT_C_STRUCT:
			{
				matvar_t **fields = (matvar_t **)matvarplus->matvar->data;
				int        nfields = matvarplus->matvar->nbytes / matvarplus->matvar->data_size;
				int        i;

				if ( matvarplus->matvar->name ) 
					Mat_Message("      Name: %s", matvarplus->matvar->name);
				Mat_Message("      Rank: %d", matvarplus->matvar->rank);
				if ( matvarplus->matvar->rank == 0 )
					return;
				Mat_Message("Class Type: Structure");
				Mat_Message("Fields[%d] {", nfields);
				indent++;
				for ( i = 0; i < nfields; i++ ) {
					matvarplus_t* temp = new matvarplus_t(fields[i],matvarplus);
					matvarplus->children.push_back(temp);
					print_default(temp);
				}
				indent--;
				Mat_Message("}");
				break;
			}
		case MAT_C_CELL:
			{
				matvar_t **cells = (matvar_t **)matvarplus->matvar->data;
				int        ncells = matvarplus->matvar->nbytes / matvarplus->matvar->data_size;
				int        i;
				if ( matvarplus->matvar->name )
					Mat_Message("      Name: %s", matvarplus->matvar->name);
				Mat_Message("      Rank: %d", matvarplus->matvar->rank);
				if ( matvarplus->matvar->rank == 0 )
					return;
				Mat_Message("Class Type: Cell Array");
				Mat_Message("{");
				indent++;
				for ( i = 0; i < ncells; i++ ) {
					matvarplus_t* temp = new matvarplus_t(cells[i],matvarplus);
					matvarplus->children.push_back(temp);
					print_default(temp);
				}
				indent--;
				Mat_Message("}");
				break;
			}
		default:
			Mat_Message("Empty");
		}
	}
}
#endif
