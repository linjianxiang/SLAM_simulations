%function L = landmarks_generate(l,n,random_landmark)
function L = landmarks_generate(map)
l = map.map_length;
n = map.landmark_number;
random_landmark = map.random_landmark;
% l : size of the map will be l x l
% n : number of lanmarks, landmarks will be randomly generated within the map
% L : landmarks location 
% random_landmark: %true for random_landmark, false for fixed landmark
if random_landmark == true %random generate
    %LAND MARK GENERATION
    L = zeros(2,n);
    Lmin=-l;
    Lmax=l;
    L(1,:)= Lmin+rand(1,n)*(Lmax-Lmin);
    L(2,:)= Lmin+rand(1,n)*(Lmax-Lmin);
else %for debug, fixed landmark
%     L = [0.2569    9.7111    2.9760    5.2507    8.6234    8.9640    1.8901    6.6072    9.4123    9.7571;
%     1.0794    1.7890    7.4655    0.4947    0.7128    4.8913    8.4989    9.9704    0.0439    5.4261];

L = [-2.9216    8.3048   -1.6630   -4.3855    5.1116    2.2949   -1.8514   -5.0406   -2.8144    8.3783   -7.3948 7.0292   -8.5120    4.8545   -1.4425    2.5802    8.3021   -5.1813   -2.0811    9.0955;
    -0.1186    0.6437    7.0577   -3.5748   -9.4220    3.9106   -9.1060   -3.9922   -8.8822    0.0077   -3.4033 -9.3661   -5.9130    7.4274   -5.6249    4.5262    0.4374    2.6561    3.1991    0.861];
end


end