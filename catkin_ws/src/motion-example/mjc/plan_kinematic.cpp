// Plan for a MuJoCo environment with OMPL

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
// #include <chrono>

#include <ompl/geometric/SimpleSetup.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/VFRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/TSRRT.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>

#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/est/BiEST.h>

#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>

#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>

#include <ompl/geometric/planners/stride/STRIDE.h>

#include <ompl/geometric/planners/pdst/PDST.h>

#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>

#include <ompl/geometric/planners/quotientspace/QRRT.h>

#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
// #include <ompl/geometric/planners/prm/SPARStwo.h>

#include <ompl/geometric/planners/sst/SST.h>




#include <yaml-cpp/yaml.h>
#include "mujoco_wrapper.h"
#include "mujoco_ompl_interface.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace std;


int main(int argc, char** argv) {
    cout << "[plan_kinematic]: WE HERE" << endl;

    string xml_filename = "";
    string prob_config_filename = "";
    string algo;
    double range_val;
    if (argc >= 3) {
        xml_filename = argv[1];
        prob_config_filename = argv[2];
    } else {
        cerr << "Format: plan_ompl_kinematic <MuJoCo XML config> <yaml problem spec> [time limit]" << endl;
        return -1;
    }

    // Optional time limit
    double timelimit = 10.0;
    if (argc >= 4) {
        stringstream ss;
        ss << argv[3];
        ss >> timelimit;
    }

    // while(true){
    

    cout << "[plan_kinematic]: Loaded Config [" << prob_config_filename << "]" << endl;
    cout << "[plan_kinematic]: Loaded Config [" << prob_config_filename << "]" << endl;

    // Create MuJoCo Object
    string mjkey_filename = strcat(getenv("HOME"), "/.mujoco/mjkey.txt");
    auto mj(make_shared<MuJoCo>(mjkey_filename));

    // Get xml file name
    if (xml_filename.find(".xml") == string::npos) {
        cerr << "XML model file is required" << endl;
        return -1;
    }

    // Load Model
    // cout << "Loading MuJoCo config from: " << xml_filename << endl;
    if (!mj->loadXML(xml_filename)) {
        cerr << "Could not load XML model file" << endl;
        return -1;
    }
    // Make data
    if (!mj->makeData()) {
        cerr << "Could not allocate mjData" << endl;
        return -1;
    }
    // cout << "[plan_kinematic]: Made Data" << endl;
        // Load yaml information
        //   This should contain instructions on how to setup the planning problem
        vector<double> start_vec;
        vector<double> goal_vec;
        map<string, int> planner_map {
            {"prm", 1}, {"rrtconnect", 1}, {"rrtstar", 1}, {"rrt", 1}, {"lbtrrt", 1}, {"trrt", 1}, {"prrt", 1}, {"lazyrrt", 1}, {"bitrrt", 1}, {"est", 1}, 
            {"biest", 1}, {"sbl", 1}, {"psbl", 1}, {"kpiece", 1}, {"bkpiece", 1}, {"lbkpiece", 1}, {"stride", 1}, {"pdst", 1}, {"fmt", 1}, {"bfmt", 1}, {"lazyprm", 1}, 
            {"prmstar", 1}, {"lazyprmstar", 1}, {"spars", 1}, {"sst", 1}, 
            };
        if (prob_config_filename != "") {
            cout << "[plan_kinematic]: Loading Config [" << prob_config_filename << "]" << endl;

            YAML::Node node = YAML::LoadFile("problems/fanuc_prob.yaml");
            cout << "[plan_kinematic]:  Config [" << prob_config_filename << "]" << endl;
            // cout << node << endl;s]
            // Copy variables
            if (node["start"]) {
    cout << "[plan_kinematic]: Lstart [" << prob_config_filename << "]" << endl;

                start_vec = node["start"].as<vector<double> >();
            }
            if (node["goal"]) {
    cout << "[plan_kinematic goal [" << prob_config_filename << "]" << endl;

                goal_vec = node["goal"].as<vector<double> >();
            }
            cout << "WE GOT" << endl;
            if (node["goal"]){
            algo = node["algorithm"].as<string>();
            }
            // cout << endl;
            if (planner_map[algo]){
                cout << "Specify " << algo << " range: ";
                // cin >> range_val;
                range_val = node["range"].as<double>();
        }
        // cout << start_vec << endl;
        }
        // Setup OMPL environment
        auto si = MjOmpl::createSpaceInformationKinematic(mj->m);
        // cout << "[plan_kinematic]: Created Space Information Kinematic" << endl;
        auto msvc(std::make_shared<MjOmpl::MujocoStateValidityChecker>(si, mj, false));
        // cout << "          MSVC: [" << msvc->isValid(si) << "]" << endl;
        // cout << "          MSVC: [" << msvc->getSpecs() << "]" << endl;

        si->setStateValidityChecker(msvc);
        si->setStateValidityCheckingResolution(0.1);
        // auto planner(make_shared<og::PRM>(si));
        

        cout << "Algorithm: " << algo << endl;
        // cin >> algo;
        



        // Create planner
        // auto planner(make_shared<og::PRM>(si));
        // planner->setRange(range_val);
        
        std::for_each(algo.begin(), algo.end(), [](char & c) {
            c = ::tolower(c);
        });

        si->setup();
        og::SimpleSetup ss(si);

        
        if (algo == "prm"){
            auto planner(make_shared<og::PRM>(si));
            ss.setPlanner(planner);

        } else if (algo == "rrtconnect"){
            auto planner(make_shared<og::RRTConnect>(si));
            planner->setRange(range_val);
            planner->setIntermediateStates(true);
            ss.setPlanner(planner);
        } else if (algo == "rrtstar"){
            auto planner(make_shared<og::RRTstar>(si));
            planner->setRange(range_val);
            ss.setPlanner(planner);
        } else if (algo == "rrt"){
            auto planner(make_shared<og::RRT>(si));
            planner->setRange(range_val);
            ss.setPlanner(planner);
        } else if (algo == "lbtrrt"){
            auto planner(make_shared<og::LBTRRT>(si));
            planner->setRange(range_val);
            ss.setPlanner(planner);
        } else if (algo == "trrt"){
            auto planner(make_shared<og::TRRT>(si));
            planner->setRange(range_val);
            ss.setPlanner(planner);
        } else if (algo == "prrt"){
            auto planner(make_shared<og::pRRT>(si));
            planner->setRange(range_val);
            ss.setPlanner(planner);
        } else if (algo == "lazyrrt"){
            auto planner(make_shared<og::LazyRRT>(si));
            planner->setRange(range_val);
            ss.setPlanner(planner);
        } else if (algo == "bitrrt"){
            auto planner(make_shared<og::BiTRRT>(si));
            planner->setRange(range_val);
            ss.setPlanner(planner);
        } else if (algo == "est"){
            auto planner(make_shared<og::EST>(si));
            planner->setRange(range_val);
            ss.setPlanner(planner);
        } else if (algo == "biest"){
            auto planner(make_shared<og::BiEST>(si));
            planner->setRange(range_val);
            ss.setPlanner(planner);
        } else if (algo == "sbl"){
            auto planner(make_shared<og::SBL>(si));
            planner->setRange(range_val);
            ss.setPlanner(planner);
        } else if (algo == "psbl"){
            auto planner(make_shared<og::pSBL>(si));
            planner->setRange(range_val);
            ss.setPlanner(planner);
        } else if (algo == "kpiece"){
            auto planner(make_shared<og::KPIECE1>(si));
            ss.setPlanner(planner);
        } else if (algo == "bkpiece"){
            auto planner(make_shared<og::BKPIECE1>(si));
            planner->setRange(range_val);
            ss.setPlanner(planner);
        } else if (algo == "lbkpiece"){
            auto planner(make_shared<og::LBKPIECE1>(si));
            planner->setRange(range_val);
            ss.setPlanner(planner);
        } else if (algo == "stride"){
            auto planner(make_shared<og::STRIDE>(si));
            planner->setRange(range_val);
            ss.setPlanner(planner);
        } else if (algo == "pdst"){
            auto planner(make_shared<og::PDST>(si));
            ss.setPlanner(planner);
        } else if (algo == "fmt"){
            auto planner(make_shared<og::FMT>(si));
            ss.setPlanner(planner);
        } else if (algo == "bfmt"){
            auto planner(make_shared<og::BFMT>(si));
            ss.setPlanner(planner);
        } else if (algo == "lazyprm"){
            auto planner(make_shared<og::LazyPRM>(si));
            planner->setRange(range_val);
            ss.setPlanner(planner);
        } else if (algo == "prmstar"){
            auto planner(make_shared<og::PRMstar>(si));
            ss.setPlanner(planner);
        } else if (algo == "lazyprmstar"){
            auto planner(make_shared<og::LazyPRMstar>(si));
            planner->setRange(range_val);
            ss.setPlanner(planner);
        } else if (algo == "spars"){
            auto planner(make_shared<og::SPARS>(si));
            ss.setPlanner(planner);
        } else if (algo == "sst"){
            auto planner(make_shared<og::SST>(si));
            double sst_selection_radius = -1.0;
            double sst_pruning_radius = -1.0;
            planner->setSelectionRadius(sst_selection_radius); // default 0.2
            planner->setPruningRadius(sst_pruning_radius);
            planner->setRange(0.02);
            ss.setPlanner(planner);
        }
        // cout << "[plan_kinematic]: Completed State Validity Check" << endl;
        



        
        // Set start and goal states
        ob::ScopedState<> start_ss(ss.getStateSpace());

        for(int i=0; i < start_vec.size(); i++) {
            start_ss[i] = start_vec[i];
        }

        ob::ScopedState<> goal_ss(ss.getStateSpace());
        for(int i=0; i < goal_vec.size(); i++) {
            goal_ss[i] = goal_vec[i];
        }

        ss.setStartAndGoalStates(start_ss, goal_ss);

        // Call the planner
        ob::PlannerStatus solved = ss.solve(timelimit);

        if (solved) {
            // cout << ss.getLastPlanComputationTime() << endl;;
            cout << "Found Solution with status: " << solved.asString() << endl;
            // ss.getSolutionPath().print(cout);
            cout << "Path Length: [" << ss.getSolutionPath().length() << "] Computation Time: [" << ss.getLastPlanComputationTime() << "]" << endl;
            // ss.
            // Write solution to file
            ofstream out_file;
            out_file.open("plan.out");
            ss.getSolutionPath().printAsMatrix(out_file);
            out_file.close();
            // system("./render_plan_kinematic problems/robot.xml plan.out");
        }
    // }
    return 0;
}
