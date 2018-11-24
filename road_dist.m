function d = road_dist(loc1,loc2)
% Convert angle unit from degree to radian; distance unit is mile.
lat1 = loc1(1)*pi/180;
long1 = loc1(2)*pi/180;
lat2 = loc2(1)*pi/180;
long2 = loc2(2)*pi/180;
r = 3959;

d_lat = abs(lat1 - lat2)/2;
d_long = abs(long1 - long2)/2;
sigma = 2*asin(sqrt(sin(d_lat)^2 + cos(lat1)*cos(lat2)*sin(d_long)^2));
great_circle_dist = r*sigma;
d = 1.2*great_circle_dist;
end