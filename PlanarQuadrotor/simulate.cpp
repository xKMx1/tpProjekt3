/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include "simulate.h"

Eigen::MatrixXf LQR(PlanarQuadrotor &quadrotor, float dt) {
    /* Calculate LQR gain matrix */
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 4e-3, 4e-3, 4e2, 8e-2, 4.5e-1, 2 / 2 / M_PI;
    R.row(0) << 3e1, 7;
    R.row(1) << 7, 3e1;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;
    
    return LQR(A_discrete, B_discrete, Q, R);
}

void control(PlanarQuadrotor &quadrotor, const Eigen::MatrixXf &K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

int main(int argc, char* args[])
{
    time_t start = time(0);
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;

    /**
     * TODO: Extend simulation
     * 1. Set goal state of the mouse when clicking left mouse button (transform the coordinates to the quadrotor world! see visualizer TODO list) #
     *    [x, y, 0, 0, 0, 0]
     * 2. Update PlanarQuadrotor from simulation when goal is changed #
    */
    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);
    /**
     * Goal pose for the quadrotor
     * [x, y, theta, x_dot, y_dot, theta_dot]
     * For implemented LQR controller, it has to be [x, y, 0, 0, 0, 0]
    */
    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << 0, 0, 0, 0, 0, 0;
    quadrotor.SetGoal(goal_state);
    /* Timestep for the simulation */
    const float dt = 0.001;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);

    /**
     * TODO: Plot x, y, theta over time
     * 1. Update x, y, theta history vectors to store trajectory of the quadrotor
     * 2. Plot trajectory using matplot++ when key 'p' is clicked
    */
    std::vector<double> x_history;
    std::vector<double> y_history;
    std::vector<double> theta_history;

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        SDL_Event e;
        bool quit = false;
        float delay;
        int x, y;
        float x_plot, y_plot, theta_plot;
        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);
        Eigen::Vector2f new_goal = Eigen::Vector2f::Zero(2);

        int n = 1;
        while (!quit)
        {
            Eigen::VectorXf z_plot = quadrotor.GetState();
            x_plot = z_plot[0];
            y_plot = z_plot[1];
            theta_plot = z_plot[2];
            double plot_insertion_timer = difftime(time(0), start);
            if(plot_insertion_timer >= n){
                x_history.push_back(x_plot);
                y_history.push_back(y_plot);
                theta_history.push_back(theta_plot);
                n++;
            }

            //events
            while (SDL_PollEvent(&e) != 0)
            {
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else if (e.type == SDL_MOUSEMOTION)
                {
                    SDL_GetMouseState(&x, &y);
                    std::cout << "Mouse position: (" << x << ", " << y << ")" << std::endl;
                }
                else if (e.type == SDL_MOUSEBUTTONUP){
                    goal_state << (x - 640), (y - 360), 0, 0, 0, 0;
                    quadrotor.SetGoal(goal_state);
                }
                else if (e.type == SDL_KEYUP){
                    if(e.key.keysym.sym == SDLK_p){
                        double seconds_since_start = difftime(time(0), start);\
                        std::vector<double> time_history = matplot::linspace(0,seconds_since_start);
                        std::cout << time_history.size() << std::endl;

                        matplot::tiledlayout(3, 1);
                        auto ax1 = matplot::nexttile();
                        auto p1 = matplot::plot(ax1, time_history, x_history, "-o");
                        matplot::xlabel(ax1, "time");
                        matplot::ylabel(ax1, "x");

                        auto ax2 = matplot::nexttile();
                        auto p2 = matplot::plot(ax2, time_history, y_history, "--xr");
                        matplot::xlabel(ax2, "time");
                        matplot::ylabel(ax2, "y");

                        auto ax3 = matplot::nexttile();
                        auto p3 = matplot::plot(ax3, time_history, theta_history, "-:gr");
                        matplot::xlabel(ax3, "time");
                        matplot::ylabel(ax3, "theta");

                        matplot::show();
                    }
                }
            }

            SDL_Delay((int) dt * 1000);

            SDL_SetRenderDrawColor(gRenderer.get(), 255, 255, 255, 255);
            SDL_RenderClear(gRenderer.get());

            /* Quadrotor rendering step */
            quadrotor_visualizer.render(gRenderer);

            SDL_RenderPresent(gRenderer.get());

            /* Simulate quadrotor forward in time */
            control(quadrotor, K);
            quadrotor.Update(dt);
        }
    }
    SDL_Quit();
    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 255, 255, 255, 255);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}