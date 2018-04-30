dimension = 2;
GG = XGraph(dimension);  % this one is for the maze

% low difficulty
% assign some vertices and edges
vertices = [0 0; 2 0; 3 0; 
            1 1; 3 1;
            0 2; 2 2;
            1 3; 3 3;
            0 4; 2 4;
            1 5; 3 5;
            0 6; 2 6;
            1 7; 3 7;
            0 8; 2 8;
            0 9; 1 9; 3 9;
];
edges = [   1 2; 1 6; 3 5; 4 5; 5 9; 6 7; 6 10; 8 9; 9 13; 10 11; 10 14;
            12 13; 13 17; 14 15; 14 18; 16 17; 17 22; 18 19; 18 20; 21 22];
starting_edge = [2, 3];
ending_edge = [20, 21];
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
sr = swarm_robot(10, [2.5 0.5]);
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
