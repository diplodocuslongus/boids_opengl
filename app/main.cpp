/*********************************************************************************************************************
 * File : main.cpp                                                                                                   *
 *                                                                                                                   *
 * 2020 Thomas Rouch                                                                                                 *
 *********************************************************************************************************************/

#include <vector>
#define FREEGLUT_EXT

#include <GL/freeglut.h>
//#include <GL/glut.h>
#include "camera_trackball.h"

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glut.h"
#include "imgui/imgui_impl_opengl2.h"

#include "boid.h"
#include "target.h"
#include "obstacle.h"

#include <memory>
#include <iostream>

const float FOVY = 60.0f;
const float NEARCLIP = 0.1f;
const float FARCLIP = 150.0f;
const int FPS = 30;
const int WINDOW_X = 700;
const int WINDOW_Y = 100;

// Inputs
int mouse_x = 0;
int mouse_y = 0;
int mouse_buttons[GLUT_NUM_MOUSE_BUTTONS];
int window_w = 800;
int window_h = 600;

// Camera
CameraTrackball camera;

std::vector<Boid> boids_;
std::vector<Target> targets_;
Target *crt_target_;
int crt_target_id_ = 0;

std::vector<Obstacle> obstacles_;

int boids_number = 50; // 100; //1000; 
float neighborhood_max_dist = 10.0f;
float separation_weight = 0.02f;
float alignment_weight = 0.005f;
float cohesion_weight = 0.07f;
float max_speed = 10.0f;
float target_attraction_weight = 0.02f;
float target_speed_alignment_weight = 0.03f;

// help
void printHelp() {
    std::cout << "Usage: ./bin/./main [options]" << std::endl;
    std::cout << "Boid simulation." << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -h, --help       Display this help message and exit." << std::endl;
    std::cout << "--nb_boids    Number of boids in the simulation" << std::endl;
    std::cout << "--maxdist     Maxmum distance between boids    " << std::endl;
    std::cout << "--separation  Separation weight                " << std::endl;
    std::cout << "--alignment   Alignment weight                 " << std::endl;
    std::cout << "--cohesion    Cohesion weight                  " << std::endl;
    std::cout << "--maxspeed    Maximum boid speed               " << std::endl;
    std::cout << "--targetattract Attraction to target weight    " << std::endl;
    std::cout << "--targetspeedalign Speed of alignment to target" << std::endl;
}
// --- SET THE WEIGHTS AND PARAMETERS ---
void set_boid_params(int nbboids,float maxdist,float maxspeed,float targetattract, float targetspeedalign) {
    MovingObject::setBoidNumber(nbboids);
    MovingObject::setNeighborMaxDist(maxdist);
    MovingObject::setMaxSpeed(maxspeed);
    MovingObject::setTargetAttractionWeight(targetattract);
    MovingObject::setTargetSpeedAlignmentWeight(targetspeedalign);
}
void set_boid_weights(float sep, float align, float coh) {
    MovingObject::setSeparationWeight(sep);
    MovingObject::setAlignmentWeight(align);
    MovingObject::setCohesionWeight(coh);
}
// parse command-line arguments.
int parse_command_line_args(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printHelp();
            return 1; // Exit 
        }
        else if (arg == "--nb_boids") {
            if (i + 1 < argc) {
                try {
                    boids_number = std::stoi(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --nb_boids requires an integer argument. Using default." << std::endl;
                }
            }
        } 
        // neighborhood  max distance
        else if (arg == "--maxdist") {
            if (i + 1 < argc) {
                try {
                    neighborhood_max_dist = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --maxdist requires a float argument. Using default." << std::endl;
                }
            }
        } 
        else if (arg == "--separation") {
            if (i + 1 < argc) {
                try {
                    separation_weight = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --separation requires a float argument. Using default." << std::endl;
                }
            }
        } 
        else if (arg == "--alignment") {
            if (i + 1 < argc) {
                try {
                    alignment_weight = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --alignment requires a float argument. Using default." << std::endl;
                }
            }
        } else if (arg == "--cohesion") {
            if (i + 1 < argc) {
                try {
                    cohesion_weight = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --cohesion requires a float argument. Using default." << std::endl;
                }
            }
        } 
        else if (arg == "--maxspeed") {
            if (i + 1 < argc) {
                try {
                    max_speed = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --maxspeed requires a float argument. Using default." << std::endl;
                }
            }
        }
        else if (arg == "--targetattract") {
            if (i + 1 < argc) {
                try {
                    target_attraction_weight = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --targetattract requires a float argument. Using default." << std::endl;
                }
            }
        }
        else if (arg == "--targetspeedalign") {
            if (i + 1 < argc) {
                try {
                    target_speed_alignment_weight = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --targetspeedalign requires a float argument. Using default." << std::endl;
                }
            }
        }
    }
    return 0; // 
}

// Boids are characterized by their position and speed
// set position and speed initial values
void add_boid()
{
    int scale = 1;

    float x = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float y = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float z = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

    float u = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float v = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float w = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    boids_.emplace_back(scale * Vec3f(x, y, z), 0.1 * Vec3f(u, v, w));
}

void init(void)
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);

    // Init inputs
    for (int i(0); i < GLUT_NUM_MOUSE_BUTTONS; ++i)
        mouse_buttons[i] = GLUT_UP;

    // Init camera
    camera.init({0.0f, 0.0f, 0.0f}, 30.0f);

    // for (int j = 0; j < boids_number_; j++)
    for (int j = 0; j < boids_number; j++)
        add_boid();

    const int scale = 3;
    for (double i : {-2 * scale, 2 * scale})
        targets_.emplace_back(scale * Vec3f(i, 0, 0));

    for (double k : {-9, 9})
        targets_.emplace_back(scale * Vec3f(0, 0, k));

    // define obstacle
    // for (double j : {-7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7})
    //     for (double k : {-7, -6, -5, -4, -3, -2, 2, 3, 4, 5, 6, 7})
    //         obstacles_.emplace_back(scale * Vec3f(0, j, k), scale);
    //
    // for (double j : {-7, -6, -5, -4, -3, 0, 3, 4, 5, 6, 7})
    //     for (double k : {-1, 0, 1})
    //         obstacles_.emplace_back(scale * Vec3f(0, j, k), scale);
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();

    ImGui::Begin("Test");
    int temp_boid_nb = MovingObject::getBoidNumber();
    // ImGui::SliderInt("Number of boids", &boids_number_, 0, 2000);
    ImGui::SliderInt("Number of boids", &boids_number, 0, 2000);
    MovingObject::setBoidNumber(temp_boid_nb);
    float temp_neighbor_max_dist = MovingObject::getNeighborMaxDist();
    ImGui::SliderFloat("Neighbor Distance", &temp_neighbor_max_dist, 0.0f, 12.f);
    MovingObject::setNeighborMaxDist(temp_neighbor_max_dist);
    float temp_separation_weight = MovingObject::getSeparationWeight();
    ImGui::SliderFloat("Separation", &temp_separation_weight, 0.0f, 10.0f);
    MovingObject::setSeparationWeight(temp_separation_weight);

    // ImGui::SliderFloat("Separation", &MovingObject::separation_factor_, 0.0f, 0.1f);
    float temp_cohesion_weight = MovingObject::getCohesionWeight();
    ImGui::SliderFloat("Cohesion", &temp_cohesion_weight, 0.0f, 0.1f);
    // ImGui::SliderFloat("Cohesion", &MovingObject::cohesion_factor_, 0.0f, 0.1f);
    MovingObject::setCohesionWeight(temp_cohesion_weight);
    float temp_align_weight = MovingObject::getAlignmentWeight();
    ImGui::SliderFloat("Alignment", &temp_align_weight, 0.0f, 0.02f);
    // ImGui::SliderFloat("Alignment", &MovingObject::alignment_factor_, 0.0f, 0.02f);
    MovingObject::setAlignmentWeight(temp_align_weight);
    float temp_targt_attraction_weight = MovingObject::getTargetAttractionWeight();
    // ImGui::SliderFloat("Target attraction", &Target::target_attraction_factor_, 0.0f, 1.f);
    ImGui::SliderFloat("Target attraction", &temp_targt_attraction_weight, 0.0f, 1.f);
    MovingObject::setTargetAttractionWeight(temp_targt_attraction_weight);
    float temp_targt_speed_align_weight = MovingObject::getTargetSpeedAlignmentWeight();
    ImGui::SliderFloat("Target speed attraction", &temp_targt_speed_align_weight, 0.0f, 0.05f);
    // ImGui::SliderFloat("Target speed attraction", &Target::target_speed_alignment_factor_, 0.0f, 0.05f);
    ImGui::SliderFloat("Obstacle", &Obstacle::obstacle_factor_, 0.f, 200.f);
    ImGui::SliderFloat("Randomness", &MovingObject::randomness_, 0.0f, 2.f);
    float temp_max_speed = MovingObject::getMaxSpeed();
    ImGui::SliderFloat("Max speed", &temp_max_speed, 0.0f, 20.f);
    // ImGui::SliderFloat("Max speed", &MovingObject::max_speed_, 0.0f, 20.f);
    MovingObject::setMaxSpeed(temp_max_speed);
    ImGui::SliderFloat("Min cos angle", &MovingObject::min_cos_angle_, -1.f, 1.f);

    ImGui::End();

    //Camera setup
    camera.lookAt();

    for (const auto &boid : boids_)
        boid.draw();

    if (crt_target_)
        crt_target_->draw();

    for (const auto &obstacle : obstacles_)
        obstacle.draw();

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    glutSwapBuffers();
}

void reshape(int w, int h)
{
    ImGui_ImplGLUT_ReshapeFunc(w, h);

    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(FOVY, (GLfloat)w / (GLfloat)h, NEARCLIP, FARCLIP);
}

void processKeys(unsigned char key, int x, int y)
{
    ImGui_ImplGLUT_KeyboardFunc(key, x, y);
    if (key == 32) // Space
    {
        crt_target_id_ = (crt_target_id_ + 1) % targets_.size();
        crt_target_ = &targets_[crt_target_id_];
    }
    else if (key == 'n')
    {
        boids_number++;
        // boids_number_++;
        add_boid();
    }
}

void systemEvolution()
{
    if (boids_number != boids_.size())
    {
        if (boids_number < boids_.size())
            boids_.erase(boids_.begin() + boids_number, boids_.end());
        else
        {
            const int n_new_boids = boids_number - boids_.size();
            for (size_t i = 0; i < n_new_boids; i++)
                add_boid();
        }
    }

    for (auto &boid_1 : boids_)
        for (auto &boid_2 : boids_)
            if (boid_1.get_id() != boid_2.get_id())
                if (MovingObject::are_neighbors(boid_1, boid_2))
                    boid_1.add_neighbor(boid_2);

    if (crt_target_)
        for (auto &boid : boids_)
            boid.add_neighbor(*crt_target_);

    for (const auto &obstacle : obstacles_)
        for (auto &boid : boids_)
            boid.add_neighbor(obstacle);

    const float t = (float)glutGet(GLUT_ELAPSED_TIME) * 0.001;
    for (auto &boid : boids_)
        boid.update(t);
}

void mouseButton(int button, int state, int x, int y)
{
    ImGui_ImplGLUT_MouseFunc(button, state, x, y);
    if (ImGui::GetIO().WantCaptureMouse)
        return;

    mouse_buttons[button] = state;

    // Update camera
    camera.zoom(mouse_buttons[3] - mouse_buttons[4]);
}

void mousePassiveMotion(int x, int y)
{
    mouse_x = x;
    mouse_y = y;
}

void mouseMotion(int x, int y)
{
    ImGui_ImplGLUT_MotionFunc(x, y);
    if (ImGui::GetIO().WantCaptureMouse)
        return;

    int mouse_dx = mouse_x - x;
    int mouse_dy = mouse_y - y;
    float dxn = static_cast<float>(mouse_dx) / static_cast<float>(window_w);
    float dyn = -static_cast<float>(mouse_dy) / static_cast<float>(window_h);
    mouse_x = x;
    mouse_y = y;

    // Update camera
    if (mouse_buttons[GLUT_LEFT_BUTTON] == GLUT_DOWN)
        camera.rotate(dxn, dyn);
    if (mouse_buttons[GLUT_RIGHT_BUTTON] == GLUT_DOWN)
        camera.pan(dxn, dyn, 0.0f);
}
// Mouse wheel handler (requires freeglut)
void mouseWheel(int wheel, int direction, int x, int y) {
    std::cout << "mouseWheel: wheel=" << wheel
              << " direction=" << direction
              << " x=" << x << " y=" << y << std::endl;
    float step = 5.0f; //  zoom step
    if (direction > 0) {
        camera.zoom(-step); // zoom in
    } else {
        camera.zoom(step);  // zoom out
    }
    glutPostRedisplay();
}

void timer(int v)
{
    glutPostRedisplay();
    glutTimerFunc(1000 / FPS, timer, 0);
}

// Main function: GLUT runs as a console application
int main(int argc, char **argv)
{
    if (argc < 2) {
        std::cout << "Using default parameters. Use -h or --help for usage." << std::endl;
        // return 1;
    }
    if (parse_command_line_args(argc, argv) == 1) return 0;
    set_boid_params(boids_number,neighborhood_max_dist,max_speed,target_attraction_weight ,target_speed_alignment_weight);
    set_boid_weights( separation_weight, alignment_weight, cohesion_weight);

    // Init GLUT and create window
    glutInit(&argc, argv);
    glutInitWindowSize(window_w, window_h);
    glutInitWindowPosition(WINDOW_X, WINDOW_Y);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glViewport(0, 0, window_w, window_h);
    glutCreateWindow("Test window");

    init();

    // Register callbacks
    glutDisplayFunc(display);
    glutIdleFunc(systemEvolution);
    glutTimerFunc(1000 / FPS, timer, 0);

    // Setup Dear ImGui context
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGLUT_Init();
    ImGui_ImplGLUT_InstallFuncs();
    ImGui_ImplOpenGL2_Init();

    // Replace some handlers after ImGui_ImplGLUT_InstallFuncs() sets its own
    // our impls will call the Imgui impls internally
    glutPassiveMotionFunc(mousePassiveMotion);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouseButton);
    glutKeyboardFunc(processKeys);
    glutMotionFunc(mouseMotion);
#ifdef FREEGLUT
    glutMouseWheelFunc(mouseWheel);
#endif
 
    glutMainLoop();

    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGLUT_Shutdown();
    ImGui::DestroyContext();
    return 0;
}
