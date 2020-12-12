//void vincenty() {
//  double a = 6378137, b = 6356752.314245, f = 0.00335281066474748071984552861852;//1 / 298.257223563;
//  double L = rad(wpLon - Lon);
//  double u1 = atan((1 - f) * tan(rad(Lat)));
//  double u2 = atan((1 - f) * tan(rad(wpLat)));
//  double sinu1 = sin(u1), cosu1 = cos(u1);
//  double sinu2 = sin(u2), cosu2 = cos(u2);
//  double cosSqAlpha, sinSigma, cos2Sigma, cosSigma, sigma;
//  double lambda = L, lambdaP, iterlimit = 1000;
//  double sinLambda = sin(lambda), cosLambda = cos(lambda);
//  do {
//
//    sinSigma = sqrt((cosu2 * sinLambda) * (cosu2 * sinLambda) + (cosu1 * sinu2 - sinu1 * cosu2 * cosLambda) * (cosu1 * sinu2 - sinu1 * cosu2 * cosLambda));
//
//    //if(sinSigma == 0) return 0;
//    cosSigma = sinu1 * sinu2 + cosu1 * cosu2 * cosLambda;
//    sigma = atan2(sinSigma, cosSigma);
//    double sinAlpha = cosu1 * cosu2 * sinLambda / sinSigma;
//    cosSqAlpha = 1 - sinAlpha * sinAlpha;
//    cos2Sigma = cosSigma - 2 * sinu1 * sinu2 / cosSqAlpha;
//    double C = f / 16 * cosSqAlpha * (4 + f * (4 - 3 * cosSqAlpha));
//    lambdaP = lambda;
//    lambda = L + (1 - C) * f * sinAlpha * (sigma + C * sinSigma * (cos2Sigma + C * cosSigma * (-1 + 2 * cos2Sigma * cos2Sigma)));
//  } while ((abs(lambda - lambdaP) > 1e-12) && (--iterlimit > 0));
//
//  //if(iterlimit == 0) return 0;
//  double uSq = cosSqAlpha * (a * a - b * b) / (b * b);
//  double A = 1 + uSq / 16384 * (4096 + uSq * (-768 + uSq * (320 - 175 * uSq)));
//  double B = uSq / 1024 * (256 + uSq * (-128 + uSq * (74 - 47 * uSq)));
//  double deltaSigma = B * sinSigma * (cos2Sigma + B / 4 * (cosSigma * (-1 + 2 * cos2Sigma) - B / 6 * cos2Sigma * (-3 + 4 * sinSigma * sinSigma) * (-3 + 4 * cos2Sigma * cos2Sigma)));
//  s = b * A * (sigma - deltaSigma);//jarak
//  a1 = atan2(cosu2 * sinLambda , cosu1 * sinu2 - sinu1 * cosu2 * cosLambda);
//  a2 = atan2(cosu1 * sinLambda , -sinu1 * cosu2 + cosu1 * sinu2 * cosLambda);
//
//  bearing = deg(a2);
//  if (bearing < 0 ) bearing += 360;
//}

rumus vincenty(){
  rumus vin;
  double a1, a2;
  double a = 6378137, b = 6356752.314245, f = 0.00335281066474748071984552861852;//1 / 298.257223563;
  double L = rad(wpLon - Lon);
  double u1 = atan((1 - f) * tan(rad(Lat)));
  double u2 = atan((1 - f) * tan(rad(wpLat)));
  double sinu1 = sin(u1), cosu1 = cos(u1);
  double sinu2 = sin(u2), cosu2 = cos(u2);
  double cosSqAlpha, sinSigma, cos2Sigma, cosSigma, sigma;
  double lambda = L, lambdaP, iterlimit = 1000;
  double sinLambda = sin(lambda), cosLambda = cos(lambda);
  do {

    sinSigma = sqrt((cosu2 * sinLambda) * (cosu2 * sinLambda) + (cosu1 * sinu2 - sinu1 * cosu2 * cosLambda) * (cosu1 * sinu2 - sinu1 * cosu2 * cosLambda));

    //if(sinSigma == 0) return 0;
    cosSigma = sinu1 * sinu2 + cosu1 * cosu2 * cosLambda;
    sigma = atan2(sinSigma, cosSigma);
    double sinAlpha = cosu1 * cosu2 * sinLambda / sinSigma;
    cosSqAlpha = 1 - sinAlpha * sinAlpha;
    cos2Sigma = cosSigma - 2 * sinu1 * sinu2 / cosSqAlpha;
    double C = f / 16 * cosSqAlpha * (4 + f * (4 - 3 * cosSqAlpha));
    lambdaP = lambda;
    lambda = L + (1 - C) * f * sinAlpha * (sigma + C * sinSigma * (cos2Sigma + C * cosSigma * (-1 + 2 * cos2Sigma * cos2Sigma)));
  } while ((abs(lambda - lambdaP) > 1e-12) && (--iterlimit > 0));

  //if(iterlimit == 0) return 0;
  double uSq = cosSqAlpha * (a * a - b * b) / (b * b);
  double A = 1 + uSq / 16384 * (4096 + uSq * (-768 + uSq * (320 - 175 * uSq)));
  double B = uSq / 1024 * (256 + uSq * (-128 + uSq * (74 - 47 * uSq)));
  double deltaSigma = B * sinSigma * (cos2Sigma + B / 4 * (cosSigma * (-1 + 2 * cos2Sigma) - B / 6 * cos2Sigma * (-3 + 4 * sinSigma * sinSigma) * (-3 + 4 * cos2Sigma * cos2Sigma)));
  vin.dis = b * A * (sigma - deltaSigma);//jarak
  a1 = atan2(cosu2 * sinLambda , cosu1 * sinu2 - sinu1 * cosu2 * cosLambda);
  a2 = atan2(cosu1 * sinLambda , -sinu1 * cosu2 + cosu1 * sinu2 * cosLambda);

  vin.azimuth = deg(a2);

  return vin;
}

float haversine() {
  float x1 = rad(wpLat - Lat);
  float yI = rad(wpLon - Lon);

  float a = sin(x1 / 2) * sin(x1 / 2) + cos(rad(Lat)) * cos(rad(wpLat)) * sin(yI / 2) * sin(yI / 2);
  float c = 2 * asin(sqrt(a));
  float d = 6371000 * c; //Nilai jarak/Haversine
  return d;
}

double rad(float val) {
  double convertRad = val * 0.0174532925;

  return convertRad;
}

double deg(float val) {
  double convertDeg = val * 57.2957795;

  return convertDeg;
}

//float havbearing() {
//  float bearingDeg, bearingRad;
//  float Lat1 = rad(Lat);
//  float Lon1 = rad(Lon);
//  float wpLat1 = rad(wpLat);
//  float wpLon1 = rad(wpLon);
//
//  float dLon = wpLon1 - Lon1;
//  y = sin(dLon) * cos(wpLat1);
//  x = cos(Lat1) * sin(wpLat1) - sin(Lat1) * cos(wpLat1) * cos(dLon);
//  bearingRad = atan2(y, x);
//  bearingDeg = deg(bearingRad);
//
//  if (bearingDeg < 0) bearingDeg += 360;
//  //if(bearingDeg>360) bearingDeg -= 360;
//  return bearingDeg;
//}
