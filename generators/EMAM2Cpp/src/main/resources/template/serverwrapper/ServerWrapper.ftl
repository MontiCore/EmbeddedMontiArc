<#-- (c) https://github.com/MontiCore/monticore -->
<#include "/Common.ftl">
#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "spdlog/spdlog.h"
#include <armadillo>
#include <grpcpp/grpcpp.h>
#include "model.grpc.pb.h"
#include "${viewModel.mainModelName}.h"

using arma::mat;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using model::Actuation;
using model::Model;
using model::ModelInputs;
using model::ModelOutputs;
using model::SensorData;
using model::Trajectory;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
using std::chrono::time_point;

Actuation make_actuation(double engine, double steering_angle, double brakes)
{
  Actuation p;
  p.set_engine(engine);
  p.set_steering_angle(steering_angle);
  p.set_brakes(brakes);
  return p;
}

class ServiceImplementation final : public Model::Service
{
private:
  time_point<high_resolution_clock> start_time_;
  int number_of_calls_;
  const int calls_stats_frequency_;

  std::shared_ptr<spdlog::logger> logger_;

  ${viewModel.mainModelName} model_;

  void pass_inputs_to_model_(const ModelInputs *request)
  {
    model_.timeIncrement = request->time_increment();
    if (request->has_sensor_data())
    {
      pass_sensor_data_(request->sensor_data());
    }
    else
    {
      logger_->warn("no sensor data is provided");
    }
    if (request->has_actuation())
    {
      pass_actuation_(request->actuation());
    }
    else
    {
      logger_->warn("no actuation is provided");
    }
    if (request->has_trajectory())
    {
      pass_trajectory_(request->trajectory());
    }
    else
    {
      logger_->warn("no trajectory is provided");
    }
  }

  void pass_sensor_data_(const SensorData &sensor_data)
  {
    model_.currentVelocity = sensor_data.velocity();
    if (sensor_data.has_position())
    {
      auto p = sensor_data.position();
      model_.x = p.x();
      model_.y = p.y();
    }
    else
    {
      logger_->warn("no position is provided");
    }
    model_.compass = sensor_data.compass();
  }

  void pass_actuation_(const Actuation &actuation)
  {
    model_.currentEngine = actuation.engine();
    model_.currentSteering = actuation.steering_angle();
    model_.currentBrakes = actuation.brakes();
  }

  void pass_trajectory_(const Trajectory &trajectory)
  {
    auto len = trajectory.points_size();
    model_.trajectory_length = len;
    if (len > 0)
    {
      mat &xs = model_.trajectory_x;
      mat &ys = model_.trajectory_y;
      for (auto i = 0; i < len; i++)
      {
        auto p = trajectory.points(i);
        xs(0, i) = p.x();
        ys(0, i) = p.y();
      }
    }
    else
    {
      logger_->warn("trajectory is empty");
    }
  }

  void log_call_()
  {
    number_of_calls_++;
    if (number_of_calls_ % calls_stats_frequency_ == 0)
    {
      auto duration = duration_cast<milliseconds>(high_resolution_clock::now() - start_time_);
      logger_->info("average response time = {} milliseconds", duration.count());
      number_of_calls_ = 0;
      start_time_ = high_resolution_clock::now();
    }
  }

public:
  ServiceImplementation() : calls_stats_frequency_(10000), start_time_(high_resolution_clock::now())
  {
    number_of_calls_ = 0;
    logger_ = spdlog::basic_logger_mt("ServiceImplementation", "/var/www/html/ServiceImplementation.log");
    model_.init();
  }

  Status Execute(ServerContext *context, const ModelInputs *request, ModelOutputs *response) override
  {
    log_call_();
    pass_inputs_to_model_(request);
    model_.execute();
    auto engine = model_.engine;
    auto steering_angle = model_.steering;
    auto brakes = model_.brakes;
    response->mutable_actuation()->CopyFrom(make_actuation(engine, steering_angle, brakes));
    return Status::OK;
  }
};

void run_server()
{
  std::cout << "initializing..." << std::endl
            << std::flush;
  ServerBuilder builder;
  std::string server_address("0.0.0.0:10247");
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  ServiceImplementation service;
  builder.RegisterService(&service);
  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "server is listening on " << server_address << std::endl
            << std::flush;
  server->Wait();
}

int main(int argc, char **argv)
{
  run_server();
  return 0;
}
