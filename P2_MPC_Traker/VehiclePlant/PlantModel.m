classdef PlantModel < handle
    properties
        m
        Iz
        Caf
        Car
        lf
        lr
    end
    
    methods
        function obj = PlantModel()
            % Constructor to initialize the properties with parameters
            params = fVehParameter();
            obj.m = params.m;
            obj.Iz = params.Iz;
            obj.Caf = params.Caf;
            obj.Car = params.Car;
            obj.lf = params.lf;
            obj.lr = params.lr;
        end
        
        function dx = computeDynamics(obj, t, states, U, x_dot)
            X = states(1);
            Y = states(2);
            y_dot = states(3);
            psi = states(4);
            psi_dot = states(5);
            
            
            % Inputs:
            delta = U;

            Fyf = obj.Caf * (delta - y_dot / x_dot - obj.lf * psi_dot / x_dot);
            Fyr = obj.Car * (-y_dot / x_dot + obj.lr * psi_dot / x_dot);

            % The nonlinear equations describing the dynamics
            dx(1, 1) = x_dot * cos(psi) - y_dot * sin(psi);
            dx(2, 1) = x_dot * sin(psi) + y_dot * cos(psi);
            dx(3, 1) = (Fyf * cos(delta) + Fyr) / obj.m - psi_dot * x_dot;
            dx(4, 1) = psi_dot;
            dx(5, 1) = (Fyf * obj.lf * cos(delta) - Fyr * obj.lr) / obj.Iz;
        end

        function [T, X] = solveDynamics(obj, tspan, initialStates, U, x_dot)
            % Solve the dynamics using ode45
            % tspan: Time span [t_start, t_end]
            % initialStates: Initial states of the system
            % U: Control input
            % x_dot: Velocity in x direction

            odeFunc = @(t, states) obj.computeDynamics(t, states, U, x_dot);
            [T, X] = ode45(odeFunc, tspan, initialStates);
        end
    end
end
