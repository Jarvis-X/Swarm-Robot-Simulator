% MIT License
%
% Copyright (c) 2018 Jarvis. X.
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

% This program makes use of the MapNested by RolandRitt and Matlab Robotics
% Toolbox by Peter Corke

classdef swarm_robot < handle
    % the class for the whole swarm of robots
    properties (Access = public)
        Size;
        initialpositions;
        positions;
        knownpath;  % a nested map which records the paths one meta robot went
        knowndeath; % which is the known edges in the maze for the swarm
        complete;
        field;
    end
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % constructor: val is the number of meta robots in the swarm
        % starting_point is the enter to the maze
        function obj = swarm_robot(val, starting_point)
            if isnumeric(val)
                obj.complete = 0;
                obj.Size = val;
                obj.positions = zeros(1,2,val);
                obj.initialpositions = zeros(1,2,val);
                obj.knownpath = MapNested();
                obj.field = containers.Map();
                for i = 1:val
                    % every robot starts from a random place on the edge
                    rx = 0.75*(rand-0.5);
                    ry = 0.75*(rand-0.5);
                    x = starting_point(1)+rx;
                    y = starting_point(2)+ry;
                    obj.positions(:,:,i) = [ x, y ];
                    obj.initialpositions(:,:,i) = [ x, y ];
                    % a temp variable stores the 1x1 block of the meta robot
                    current_block = [floor([x, y]), ceil([x y])];
                    % The explored 1x1 block gets a value on the potential
                    % field
                    if obj.field.isKey(int2str(current_block))
                        obj.field(int2str(current_block)) = obj.field(int2str(current_block)) + 0.5;
                    else
                        obj.field(int2str(current_block)) = 0.5;
                    end
                    patch('vertices', [floor([x, y]); ceil(x), floor(y); ceil([x, y]); floor(x), ceil(y)], ...
                        'faces', [1, 2, 3, 4], ...
                        'FaceColor', 'b', ...
                        'FaceAlpha', 0.005);
                    drawnow
                end
            else
                error('Value must be numeric');
            end
        end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% For debug: Property accesses
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % get the number of meta robots
        function s = getSize(obj)
            s = obj.Size;
        end

        % get the positions of meta robots
        function Ps = getposition(obj)
            Ps = zeros(obj.getSize(), 2);
            for i = 1:size(obj.positions, 3)
                Ps(i,:) = obj.positions(:,:,i);
            end
        end

        % get the potential fields of the known parts of the maze
        function [Fs_k, Fs_v] = getfields(obj)
            keySet = keys(obj.field);
            Fs_k = string(zeros(1, size(keySet,2)));
            Fs_v = zeros(1, size(keySet,2));
            for i = 1:size(Fs_v, 2)
                Fs_k(i) = string(keySet(i));
                Fs_v(i) = obj.field(char(keySet(i)));
            end
        end
        
        % get the path of a given meta robot
        function path = show_path(obj, index)
            if index == 0
                path = [];
                for i = 1:obj.getSize()
                    Xs = zeros(1, obj.knownpath(i).size(1)+1);
                    Ys = zeros(1, obj.knownpath(i).size(1)+1);
                    current_point = obj.initialpositions(:,:,i);
                    Xs(1) = current_point(1);
                    Ys(1) = current_point(2);
                    for k = 2:(obj.knownpath(i).size(1)+1)
                        current_point = obj.knownpath(i, num2str(current_point));
                        Xs(k) = current_point(1);
                        Ys(k) = current_point(2);
                    end
                    % path(i) = [Xs', Ys'];
                    plot(Xs,Ys,'-o');
                end
            else
                path = zeros(obj.knownpath(index).size(1)+1 ,2);
                current_point = obj.initialpositions(:, :, index);
                path(1,:) = current_point; 
                for k = 2:(obj.knownpath(index).size(1)+1)
                    current_point = obj.knownpath(index, num2str(current_point));
                    path(k, :) = current_point;
                end
                plot(path(:,1)', path(:,2)', '-o');
            end
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% this method does not work for now       
%         % show the maze the swarm robot knows
%         function showyourworld(obj, graph)
%             fr = figure('Name','Known Maze','NumberTitle','off');
%             % plot the edges
%             for i = 1:size(obj.knowndeath, 1)
%                 theedge = obj.knowndeath(i,:);
%                 startp = graph.vertices(:, theedge(1));
%                 endp = graph.vertices(:, theedge(2));
%                 plot([startp(1), endp(1)], [startp(2), endp(2)], 'b', 'LineWidth', 6);
%             end
%         end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Make every meta robot do a random move according to the potential
        % field, the goal is the index of the exit edge in the graph
        % the function returns the status if the robot has found the exit
        function [robot_index] = Move(obj, graph)
            robot_index = -1;
            s = obj.Size;
            for i = 1:s
                startpoint = obj.positions(:,:,i);
                % Each meta robot has the chance of moving for 5 times
                % for every iteration
                % the flag records a single collision
                flag = 0;
                for j = 1:5
                    flag = 0;
                    % let's take a pair of random numbers
                    theta = 2*pi*rand;
                    d = abs(rand);
                    % then use it to generate the intending end point
                    endpoint = startpoint + [d*cos(theta), d*sin(theta)];
                    % check if the intending path conflicts with existing
                    % edge in the maze
                    for k = 1:graph.ne
                        edgestartp = (graph.vertices(:, graph.edges(1, k)))';
                        edgeendp = (graph.vertices(:, graph.edges(2, k)))';
                        % if no overlap, check other edges
                        if ~isOverlap(startpoint, endpoint, edgestartp, edgeendp)
                            continue;
                        else
                            % if there is overlap, see if it is the exit
                            if k == graph.exit
                                robot_index = i;
                                flag = 0;
                                break;
                            else
                                flag = 1;
                                nm = 0;
                                for mn = 1:size(obj.knowndeath, 2)
                                    if isequal(graph.edges(:,k)', obj.knowndeath(mn))
                                        nm = 1;
                                    end
                                end
                                if nm == 0
                                    obj.knowndeath = [obj.knowndeath; graph.edges(:, k)'];
                                end
                                break;
                            end
                        end
                    end
                    % if collision detected, try one more time
                    if flag
                        continue;
                    else
                        % store the 1x1 block of the meta robot brfore the movement
                        current_block = [floor(startpoint), ceil(startpoint)];
                        if robot_index == -1
                            % calculate next block the robot is going to explore
                            next_block = [floor(endpoint), ceil(endpoint)];
                            % only if the field does not exist or the field value is
                            % smaller than the current one can the robot move to it
                            if not(obj.field.isKey(int2str(next_block))) || obj.field(int2str(next_block)) < obj.field(int2str(current_block))
                                % update known path
                                obj.knownpath(i, num2str(startpoint)) = endpoint;
                                % update position
                                obj.positions(:,:,i) = endpoint;
                                % initialize field value for next block
                                % The 1x1 block gets a value on the potential field
                                if obj.field.isKey(int2str(next_block))
                                    obj.field(int2str(next_block)) = obj.field(int2str(next_block)) + 0.5;
                                else
                                    obj.field(int2str(next_block)) = 0.5;
                                end
                                x = endpoint(1);
                                y = endpoint(2);
                                patch('vertices', [floor([x, y]); ceil(x), floor(y); ceil([x, y]); floor(x), ceil(y)], ...
                                    'faces', [1, 2, 3, 4], ...
                                    'FaceColor', 'b', ...
                                    'FaceAlpha', 0.01);
                                drawnow
                                break;
                            else
                                % if the next field value is too large for the robot
                                % to move, increase the field value of the current block
                                obj.field(int2str(current_block)) = obj.field(int2str(current_block)) + 0.5;
                            end
                        else
                            % update known path
                            obj.knownpath(i, num2str(startpoint)) = endpoint;
                            % update position
                            obj.positions(:,:,i) = endpoint;
                            break;
                        end
                    end
                end
                % if after 5 trials, there still does not exist a path, just raise
                % the field value
                if flag
                    current_block = [floor(startpoint), ceil(startpoint)];
                    if obj.field.isKey(int2str(current_block))
                        obj.field(int2str(current_block)) = obj.field(int2str(current_block)) + 0.5;
                    else
                        obj.field(int2str(current_block)) = 0.5;
                    end
                end
                % if the exit is found, break the loop, return the robot index
                if robot_index ~= -1
                    break;
                end
            end
        end
    end
end


% the tiny function checks if the path overlaps with the existing edges
function [ flag ] = isOverlap ( sp, ep, edgesp, edgeep )
    if (dot( cross([edgesp-sp, 0], [edgeep-sp, 0]), cross([edgesp-ep, 0], [edgeep-ep, 0])) <= 0) && ...
       (dot( cross([sp-edgesp, 0], [ep-edgesp, 0]), cross([sp-edgeep, 0], [ep-edgeep, 0])) <= 0)
        flag = 1;
    else
        flag = 0;
    end
end

