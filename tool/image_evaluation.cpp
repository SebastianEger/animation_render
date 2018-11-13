#include <templateevaluation.h>

int main( int argc, char** argv )
{
    double grad_t = 0;
    double acc_t = 0;
    if(argv[2]) grad_t = atof(argv[2]);
    if(argv[3]) acc_t = atof(argv[3]);
    TemplateEvaluation templateEvaluation = TemplateEvaluation(grad_t, acc_t);
    std::string file = std::string(argv[1]);

    templateEvaluation.evaluate5(file, 1);
    file.erase( file.end()-4, file.end() );
    templateEvaluation.exportData("/home/sebastian/template_evaluation", file);
    return 0;
}
