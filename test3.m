dimension = 2;
GG = XGraph(dimension);  % this one is for the maze

% low difficulty
% assign some vertices and edges
vertices = [0 0; 3 0; 4 0; 7 0; 
            1 1; 3 1; 4 1; 6 1;
            1 2; 2 2; 3 2; 5 2;
            1 3; 3 3; 4 3; 6 3;
            1 4; 3 4; 6 4;
            0 5; 3 5; 4 5; 7 5;
];
edges = [   1 2; 1 20; 3 4; 3 7; 4 23; 5 9; 5 6; 6 11; 7 8; 8 16; 9 10;
            11 12; 11 14; 13 14; 13 17; 15 16; 17 18; 18 19; 18 21; 20 21; 22 23];
starting_edge = [2, 3];
ending_edge = [21, 22];
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
sr = swarm_robot(10, [3.5 0.5]);
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
