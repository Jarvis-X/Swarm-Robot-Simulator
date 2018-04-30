dimension = 2;
GG = XGraph(dimension);  % this one is for the maze

% medium difficulty
% assign some vertices and edges
vertices = [0 0; 5 0; 
            1 1; 4 1;
            1 2; 2 2; 3 2;
            2 3; 3 3;
            1 4; 4 4;
            0 5; 1 5; 2 5; 3 5; 5 5;
];
edges = [   1 2; 1 12; 2 16; 3 4; 4 11; 5 6; 5 10; 6 8; 7 9; 10 11;
            10 13; 12 13; 13 14; 15 16;];
starting_edge = [6, 7];
ending_edge = [14, 15];
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
sr = swarm_robot(10, [2.5 2.5]);
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
