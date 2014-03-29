#include "mattest.h"
using namespace std;
using namespace MatIO;
int main(int argc, char** argv) {
	// setup ublas

	int indent = 0;
	char program_name[] = "Matrix Test";
	mat_t *mat;
	matvar_t *matvar;
	Mat_LogInit(program_name);
	if (argc < 2)
		Mat_Error("Must specify at least one argument");
	mat = Mat_Open(argv[1], MAT_ACC_RDONLY);
	if (NULL == mat)
		Mat_Error("Error opening %s\n", argv[1]);
	while ( (matvar = Mat_VarReadNext(mat)) != NULL) {
		matvarplus_t* root = new matvarplus_t(matvar,NULL);
		print_default(root);
		print_bfstree(root);
		Eigen::MatrixXf t;
		get_matrix(t,root, "modelrgbkdes->svm->maxvalue");
		for (int i = 0; i < t.rows(); i++) {
			for (int j = 0; j < t.cols(); j++) {
				printf("%f\n", t(i,j));
			}

		}
		vector<string>* c = get_charlist(root, "modelrgbkdes->svm->classname");
		if (c == NULL)
			printf("c is NULL");
		BOOST_FOREACH(string s, *c) {
			printf("%s \n", s.c_str());
		}
		if (c == NULL)
			printf("c is NULL");
		printf("\n");
		//Mat_VarFree(matvar);
		matvar = NULL;
	}
}
