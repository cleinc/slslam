function H = house(x,y,z,l,w,h)

switch nargin
    case 0
        [x,y,z] = deal(0);
        l = 5;
        w = 5;
        h = 4;
    case {1,2}
        error('??? House position not fully specified')
    case 3
        l = 5;
        w = 5;
        h = 4;
    case {4,5}
        error('??? House dimensions not fully specified')
end

l = 4.5;
w = 4.5;
h = 3.5;

a = .2;
b = .4;
c = .6;
d = .8;

p = .25;
q = .5;
r = .65;

%H = zeros(6,23);
H = zeros(6, 74);
% 4 walls
H(:, 1)    = zSegment   (x  ,y  ,z,z+r*h); 
H(:, 2)    = zSegment   (x+l,y  ,z,z+r*h);
H(:, 3)    = zSegment   (x+l,y+w,z,z+r*h);
H(:, 4)    = zSegment   (x  ,y+w,z,z+r*h);

% floor
H(:,5:8)   = XYRectangle(x,x+l,y,y+w,z); 

% roof slopes
H(:, 9)    = makeSegment([x;y    ;z+r*h],[x;y+w/2;z+h    ]); 
H(:,10)    = makeSegment([x;y+w/2;z+h    ],[x;y+w  ;z+r*h]);
H(:,11)    = makeSegment([x+l;y    ;z+r*h],[x+l;y+w/2;z+h    ]); 
H(:,12)    = makeSegment([x+l;y+w/2;z+h    ],[x+l;y+w  ;z+r*h]);

% roof
H(:,13)    = xSegment   (x,x+l   ,y+.5*w,z+h);   % top
H(:,14)    = xSegment   (x,x+l   ,y     ,z+r*h); % lower ends
H(:,15)    = xSegment   (x,x+l   ,y+w   ,z+r*h); 

% door
H(:,16:19) = YZRectangle(x, y+c*w, y+d*w, z, z+q*h); 

% window
H(:,20:23) = YZRectangle(x,y+a*w,y+b*w,z+p*h,z+q*h); 


%
H(:, 24) = ySegment(x, y, y+w, z+r*h);
H(:, 25) = ySegment(x+l, y, y+w, z+r*h);

H(:, 26) = ySegment(x, y+a*w, y+b*w, (z+p*h + z+q*h)/2);
H(:, 27) = zSegment(x, (y+a*w + y+b*w)/2, z+p*h, z+q*h);

H(:, 28) = makeSegment([x+l/2, y, z+r*h], [x+l/2, y+w/2, z+h]);
H(:, 29) = makeSegment([x+l/4, y, z+r*h], [x+l/4, y+w/2, z+h]);
H(:, 30) = makeSegment([x+l*3/4,y, z+r*h], [x+l*3/4, y+w/2, z+h]);

H(:, 31) = makeSegment([x+l/2, y+w/2, z+h], [x+l/2, y+w, z+r*h]);
H(:, 32) = makeSegment([x+l/4, y+w/2, z+h], [x+l/4, y+w, z+r*h]);
H(:, 33) = makeSegment([x+l*3/4,y+w/2, z+h], [x+l*3/4, y+w, z+r*h]);

H(:, 34) = xSegment(x, x+l, y+w*1/8, z+r*h+(h-r*h)*1/4);
H(:, 35) = xSegment(x, x+l, y+w*2/8, z+r*h+(h-r*h)*2/4);
H(:, 36) = xSegment(x, x+l, y+w*3/8, z+r*h+(h-r*h)*3/4);

H(:, 37) = xSegment(x, x+l, y+w*5/8, z+r*h+(h-r*h)*3/4);
H(:, 38) = xSegment(x, x+l, y+w*6/8, z+r*h+(h-r*h)*2/4);
H(:, 39) = xSegment(x, x+l, y+w*7/8, z+r*h+(h-r*h)*1/4);

H(:, 40) = zSegment(x+l*1/4, y, z, z+r*h);
H(:, 41) = zSegment(x+l*2/4, y, z, z+r*h);
H(:, 42) = zSegment(x+l*3/4, y, z, z+r*h);

H(:, 43) = zSegment(x+l*1/4, y+w, z, z+r*h);
H(:, 44) = zSegment(x+l*2/4, y+w, z, z+r*h);
H(:, 45) = zSegment(x+l*3/4, y+w, z, z+r*h);

H(:, 46) = zSegment(x+l, y+w*1/4, z, z+r*h);
H(:, 47) = zSegment(x+l, y+w*2/4, z, z+r*h);
H(:, 48) = zSegment(x+l, y+w*3/4, z, z+r*h);

H(:, 49) = makeSegment([x, y+c*w, z], [x, y+d*w, z+q*h]);
H(:, 50) = makeSegment([x, y+d*w, z], [x, y+c*w, z+q*h]);

H(:, 51) = makeSegment([x, y, z], [x+1/4*l, y, z+r*h]);
H(:, 52) = makeSegment([x+1/4*l, y, z], [x, y, z+r*h]);

H(:, 53) = makeSegment([x+1/4*l, y, z], [x+2/4*l, y, z+r*h]);
H(:, 54) = makeSegment([x+2/4*l, y, z], [x+1/4*l, y, z+r*h]);

H(:, 55) = makeSegment([x+2/4*l, y, z], [x+3/4*l, y, z+r*h]);
H(:, 56) = makeSegment([x+3/4*l, y, z], [x+2/4*l, y, z+r*h]);

H(:, 57) = makeSegment([x+3/4*l, y, z], [x+l, y, z+r*h]);
H(:, 58) = makeSegment([x+l, y, z], [x+3/4*l, y, z+r*h]);

H(:, 59) = makeSegment([x+l, y, z], [x+l, y+1/4*w, z+r*h]);
H(:, 60) = makeSegment([x+l, y+1/4*w, z], [x+l, y, z+r*h]);

H(:, 61) = makeSegment([x+l, y+1/4*w, z], [x+l, y+2/4*w, z+r*h]);
H(:, 62) = makeSegment([x+l, y+2/4*w, z], [x+l, y+1/4*w, z+r*h]);

H(:, 63) = makeSegment([x+l, y+2/4*w, z], [x+l, y+3/4*w, z+r*h]);
H(:, 64) = makeSegment([x+l, y+3/4*w, z], [x+l, y+2/4*w, z+r*h]);

H(:, 65) = makeSegment([x+l, y+3/4*w, z], [x+l, y+w, z+r*h]);
H(:, 66) = makeSegment([x+l, y+w, z], [x+l, y+3/4*w, z+r*h]);

H(:, 67) = makeSegment([x, y+w, z], [x+1/4*l, y+w, z+r*h]);
H(:, 68) = makeSegment([x+1/4*l, y+w, z], [x, y+w, z+r*h]);

H(:, 69) = makeSegment([x+1/4*l, y+w, z], [x+2/4*l, y+w, z+r*h]);
H(:, 70) = makeSegment([x+2/4*l, y+w, z], [x+1/4*l, y+w, z+r*h]);

H(:, 71) = makeSegment([x+2/4*l, y+w, z], [x+3/4*l, y+w, z+r*h]);
H(:, 72) = makeSegment([x+3/4*l, y+w, z], [x+2/4*l, y+w, z+r*h]);

H(:, 73) = makeSegment([x+3/4*l, y+w, z], [x+l, y+w, z+r*h]);
H(:, 74) = makeSegment([x+l, y+w, z], [x+3/4*l, y+w, z+r*h]);
