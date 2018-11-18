function d = dist(lat1, long1, lat2, long2)
r = 3959;
d_lat = abs(lat1 - lat2);
d_long = abs(long1 - long2);
sigma = 2*asin(sqrt(sin(0.5*d_lat)^2 + cos(lat1)*cos(lat2)*sin(0.5*d_long)^2));
GCD = r*sigma;
d = 1.2*GCD;
end