#include<iostream>
#include "common/BALProblem.h"
#include "common/BundleParams.h"
#include "SnavelyReprojectionError.h"
using namespace std;
using namespace ceres;


void BuildProblem(BALProblem *bal_problem, Problem *problem, const BundleParams &params) {
    const int point_block_size = bal_problem->point_block_size();
    const int camera_block_size = bal_problem->camera_block_size();
    double *points = bal_problem->mutable_points();
    double *cameras = bal_problem->mutable_cameras();
//    observations is 2*num_observations long array observations
//    [u_1,u_2,...,u_n], where each u_i is two dimensional, the x and y position of the observation.
    const double *observations = bal_problem->observations();

    for (int i = 0; i < bal_problem->num_observations(); ++i) {
        CostFunction *cost_function;
//        each residual block takes a point and camera as input
//        and outputs a 2 dimensional residual
        cost_function = SnavelyReprojectionError::Create(observations[2 * i + 0], observations[2 * i + 1]);
//        if enabled use Huber's loss function
        LossFunction *loss_function = params.robustify ? new HuberLoss(1.0) : nullptr;
//        each observation corresponds to a pair of a camera and a point
//        which are identified by camera_index()[i] and point_index()[i] respectively
        double *camera = cameras + camera_block_size * bal_problem->camera_index()[i];
        double *point = points + point_block_size * bal_problem->point_index()[i];

        problem->AddResidualBlock(cost_function, loss_function, camera, point);
    }
}

void setOrdering(BALProblem *bal_problem, Solver::Options *options, const BundleParams &params) {
    const int num_points = bal_problem->num_points();
    const int point_block_size = bal_problem->point_block_size();
    double *points = bal_problem->mutable_points();
    const int num_cameras = bal_problem->num_cameras();
    const int cameras_block_size = bal_problem->camera_block_size();
    double *cameras = bal_problem->mutable_cameras();

    if (params.ordering == "automatic")
        return;
    auto *ordering = new ParameterBlockOrdering;
//    the points come before the cameras
    for (int i = 0; i < num_points; ++i) {
        ordering->AddElementToGroup(points + point_block_size * i, 0);
    }
    for (int i = 0; i < num_cameras; ++i) {
        ordering->AddElementToGroup(cameras + cameras_block_size * i, 1);
    }
    options->linear_solver_ordering.reset();
}

void setSolverOptionsFromFlags(BALProblem *bal_problem, const BundleParams &params, Solver::Options *options) {
    options->max_num_iterations = params.num_iterations;
    options->minimizer_progress_to_stdout = true;
    options->num_threads = params.num_threads;

//    选取下降策略
    CHECK(StringToTrustRegionStrategyType(params.trust_region_strategy, &options->trust_region_strategy_type));

//    选取linear solver
    CHECK(StringToLinearSolverType(params.linear_solver, &options->linear_solver_type));
    CHECK(StringToSparseLinearAlgebraLibraryType(params.sparse_linear_algebra_library,
                                                 &options->sparse_linear_algebra_library_type));
    CHECK(StringToDenseLinearAlgebraLibraryType(params.dense_linear_algebra_library,
                                                &options->dense_linear_algebra_library_type));

//    设置变量排序
    setOrdering(bal_problem, options, params);
}




/**
 * 本程序演示了后端ceres bundle
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {
    // set the parameters here
    BundleParams params(argc, argv);
    cout << params.input << endl;
    if (params.input.empty()) {
        cout << "Usage: ceres_bundle -input <path for dataset>";
        return 1;
    }

    BALProblem bal_problem(params.input);

    // show some information here ...
    cout << "bal problem file loaded." << endl;
    cout << "bal problem have " << bal_problem.num_cameras() << " cameras and "
         << bal_problem.num_points() << " points. " << endl;
    cout << "forming " << bal_problem.num_observations() << " observatoins. " << endl;

    // store the initial 3D cloud points and camera pose..
    if (!params.initial_ply.empty()) {
        bal_problem.WriteToPLYFile(params.initial_ply);
    }

    cout << "beginning problem." << endl;

    // add some noise for the intial value
    srand(static_cast<unsigned int>(params.random_seed));
    bal_problem.Normalize();
    bal_problem.Perturb(params.rotation_sigma, params.translation_sigma, params.point_sigma);

    cout << "normalization complete." << endl;

    Problem problem;
    BuildProblem(&bal_problem, &problem, params);

    cout << "the problem is successfully build." << endl;

    Solver::Options options;
    setSolverOptionsFromFlags(&bal_problem, params, &options);
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.FullReport() << endl;

    // write the result into a .ply file.
    if (!params.final_ply.empty()) {
        // pay attention to this: ceres doesn't copy the value into optimizer, but implement on raw data!
        bal_problem.WriteToPLYFile(params.final_ply);
    }

    return 0;
}
