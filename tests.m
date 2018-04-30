dimension = 2;
GG = XGraph(dimension);  % this one is for the maze

% lowest difficulty
% assign some vertices and edges
vertices = [0 0; 0 6; 6 0; 
            7 0; 7 6; 1 6;
            1 1; 6 1; 1 5;
            2 5; 6 5; 6 2;
            2 4; 2 2; 4 2;
            3 4; 5 4; 5 2;
            3 3; 4 3
];
edges = [1 2; 1 3; 4 5; 5 6; 7 8; 7 9; 10 11; 11 12; 13 14; 14 15; 16 17; 17 18; 19 20];
starting_edge = [3,4];
ending_edge = [2, 6];
for i = 1:size(vertices, 1)
    GG.add_node(vertices(i,:));
end
for i = 1:size(edges, 1)
    GG.add_edge(edges(i,1), edges(i,2));
end        
% add the starting and ending edges as the last two ones 
GG.add_edge(starting_edge(1), starting_edge(2));
GG.add_edge(ending_edge(1), ending_edge(2));
GG.set_exit(GG.ne);

% start the test
sr = swarm_robot(10, [6.5 0.5]);
therobot = -1;
iter = 0;
while(therobot == -1 && iter <= 1000)
    iter = iter + 1;
    disp(iter);
    tic
    therobot = sr.Move(GG);
    toc
end
if therobot ~= -1
    disp(therobot);
    sr.complete = 1;
end