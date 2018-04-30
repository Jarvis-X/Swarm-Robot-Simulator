dimension = 2;
GG = XGraph(dimension);  % this one is for the maze

% high difficulty
% assign some vertices and edges
vertices = [0 0; 3 0; 4 0; 5 0; 8 0;
            0 1; 2 1; 3 1; 5 1; 7 1;
            1 2; 4 2; 5 2; 6 2; 7 2;
            2 3; 3 3; 5 3; 6 3; 8 3;
            3 4; 4 4; 5 4; 7 4;
            2 5; 5 5; 6 5; 7 5;
            1 6; 2 6; 4 6; 5 6; 6 6; 7 6;
            1 7; 2 7; 3 7; 4 7; 5 7; 6 7;
            0 8; 3 8; 4 8; 5 8; 7 8; 8 8
];
edges = [   1 2; 1 6; 2 3; 2 8; 3 12; 4 5; 4 9; 5 20; 6 7; 6 41; 9 10; 10 15;
            11 12; 11 29; 12 13; 12 22; 14 19; 16 17; 16 25; 18 19; 18 23; 19 20;
            20 46; 21 22; 23 24; 23 26; 25 26; 26 32; 27 28; 27 33;
            29 30; 30 31; 30 36; 31 38; 32 33; 33 40; 34 45; 
            35 36; 37 42; 39 44; 
            41 42; 43 44; 44 45; 45 46;];
starting_edge = [3, 4];
ending_edge = [42, 43];
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
sr = swarm_robot(10, [4.5, 0.5]);
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
