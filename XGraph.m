classdef XGraph < handle
    % the class for the graph( Maze )
    properties (Access = public)
        ne; % number of edges
        nv; % number of vertices
        vertices; % nodes in the graph
        edges; % walls in the maze
        dimension; % dim of the graph
        exit; % the exit of the maze
        f; % the figure of the maze
    end
    methods
        % constructor
        function obj = XGraph(dimension)
            obj.ne = 0;
            obj.nv = 0;
            obj.dimension = dimension;
            obj.vertices = zeros(dimension, 0);
            obj.edges = zeros(2, 0);
            obj.exit = -1; % initial value of -1 indicates no exit
            obj.f = figure('Name','Maze','NumberTitle','off');
            hold on;
            grid on;
        end
        % add a vertex to the graph
        function [isSuccess] = add_node(obj, vertex)
            if size(vertex, 1) == 1
                vertex_add = vertex';
            else
                vertex_add = vertex;
            end
            % check if it is already in the graph
            exist = 0;
            for i = 1:size(obj.vertices, 2)
                if isequal(obj.vertices(:,i), vertex_add)
                    exist = 1;
                    break;
                end
            end
            % add the node if it does not exist
            if not(exist)
                obj.vertices = [obj.vertices, vertex_add];
                obj.nv = obj.nv + 1;
                scatter(vertex_add(1), vertex_add(2), 'filled');
                isSuccess = 1;
            else
                isSuccess = 0;
            end
        end
        % add an edge to the graph
        function [isSuccess] = add_edge(obj, startp, endp)
            % check if it is already in the graph
            exist = 0;
            for i = 1:size(obj.edges, 2)
                if (obj.edges(1,i) == startp && obj.edges(2,i) == endp) || (obj.edges(2,i) == startp && obj.edges(1,i) == endp)
                    exist = 1;
                    break;
                end
            end
            % add the edge if it does not exist
            if not(exist)
                obj.edges = [obj.edges, [startp endp]'];
                obj.ne = obj.ne + 1;
                plot([obj.vertices(1, startp), obj.vertices(1, endp)], [obj.vertices(2, startp), obj.vertices(2, endp)], 'k', 'LineWidth', 8);
                isSuccess = 1;
            else
                isSuccess = 0;
            end
        end
        % set the exit of the maze
        function [isSuccess] = set_exit(obj, edge)
            % if the exit is the edge index
            if isequal(size(edge), [1, 1]) 
                if edge <= size(obj.edges, 2)
                    obj.exit = edge;
                    startp = obj.edges(1, edge);
                    endp = obj.edges(2, edge);
                    plot([obj.vertices(1, startp), obj.vertices(1, endp)], [obj.vertices(2, startp), obj.vertices(2, endp)], 'r', 'LineWidth', 8);
                    isSuccess = 1;
                else
                    isSuccess = 0;
                end
            % if the exit is represent as two nodes
            else
                % check if the edge is in the graph
                exist = 0;
                for i = 1:size(obj.edges, 2)
                    if (obj.edges(1,i) == edge(1) && obj.edges(2,i) == edge(2)) || (obj.edges(2,i) == edge(1) && obj.edges(1,i) == edge(2))
                        exist = 1;
                        break;
                    end
                end
                if exist
                    obj.exit = i;
                    isSuccess = 1;
                else
                    isSuccess = 0;
                end
            end
        end
    end
end
