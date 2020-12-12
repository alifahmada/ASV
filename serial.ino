void recData() {
  rec();
  if (newData == true) {
    strcpy(tempChar, dataRec);
    if (parsing == 1) {
      recWaypoint();
//      Serial.print(WP[0], 7); Serial.print("|"); Serial.print(WP[1], 7); Serial.print("|");
//      Serial.print(WP[2], 7); Serial.print("|"); Serial.print(WP[3], 7); Serial.print("|");
//      Serial.print(WP[4], 7); Serial.print("|"); Serial.print(WP[5], 7); Serial.print("|");
//      Serial.print(WP[6], 7); Serial.print("|"); Serial.print(WP[7], 7); Serial.print("|");
//      Serial.print(WP[8], 7); Serial.print("|"); Serial.println(WP[9], 7);
    }
    else if (parsing == 2) {
      recSpeed();
      //Serial.println(rSpeed);
    }
    else if (parsing == 3) {
      recStart();
      //Serial.println(Start);
    }
    else if (parsing == 4) {
      recEmergency();
      //Serial.println(Emergency);
    }
    else if (parsing == 5) {
      recTuning();
      //Serial.println(Emergency);
    }
    newData = false;
  }
}

void rec() {
  static bool recData = false;
  static byte ndx;
  char endline = '>';
  char c;

  while (GCS.available() > 0 && newData == false) {
    c = GCS.read();

    if (recData == true) {
      if (c != endline) {
        dataRec[ndx] = c;
        ndx++;
        if (ndx >= dataByte) {
          ndx = dataByte - 1;
        }
      }
      else {
        dataRec[ndx] = '\0';
        recData = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (c == '!') {
      recData = true;
      parsing = 1;
      //Waypoint
    }
    else if (c == '@') {
      recData = true;
      parsing = 2;
      //Speed
    }
    else if (c == '#') {
      recData = true;
      parsing = 3;
      //Start
    }
    else if (c == '$') {
      recData = true;
      parsing = 4;
      //Emergency
    }
    else if (c == '^') {
      recData = true;
      parsing = 5;
      //Start
    }
  }
}

void recWaypoint() {
  char * strtokIndx;

  strtokIndx = strtok(tempChar, "|");
  strcpy(dataGCS, strtokIndx);

  strtokIndx = strtok(NULL, "|");
  WP[0] = atof(strtokIndx);

  strtokIndx = strtok(NULL, "|");
  WP[1] = atof(strtokIndx);

  strtokIndx = strtok(NULL, "|");
  WP[2] = atof(strtokIndx);

  strtokIndx = strtok(NULL, "|");
  WP[3] = atof(strtokIndx);

  strtokIndx = strtok(NULL, "|");
  WP[4] = atof(strtokIndx);

  strtokIndx = strtok(NULL, "|");
  WP[5] = atof(strtokIndx);

  strtokIndx = strtok(NULL, "|");
  WP[6] = atof(strtokIndx);

  strtokIndx = strtok(NULL, "|");
  WP[7] = atof(strtokIndx);

  strtokIndx = strtok(NULL, "|");
  WP[8] = atof(strtokIndx);

  strtokIndx = strtok(NULL, "|");
  WP[9] = atof(strtokIndx);

  strtokIndx = strtok(NULL, "|");
  WP[10] = atof(strtokIndx);

  strtokIndx = strtok(NULL, "|");
  WP[11] = atof(strtokIndx);

  strtokIndx = strtok(NULL, "|");
  WP[12] = atof(strtokIndx);

  strtokIndx = strtok(NULL, "|");
  WP[13] = atof(strtokIndx);

  strtokIndx = strtok(NULL, "|");
  WP[14] = atof(strtokIndx);

  strtokIndx = strtok(NULL, "|");
  WP[15] = atof(strtokIndx);

}

void recSpeed() {
  char * strtokIndx;
  strtokIndx = strtok(tempChar, "|");
  strcpy(dataGCS, strtokIndx);
  strtokIndx = strtok(NULL, "|");
  rSpeed = atoi(strtokIndx);
}

void recStart() {
  char * strtokIndx;
  strtokIndx = strtok(tempChar, "|");
  strcpy(dataGCS, strtokIndx);
  strtokIndx = strtok(NULL, "|");
  Start = atoi(strtokIndx);
}

void recEmergency() {
  char * strtokIndx;
  strtokIndx = strtok(tempChar, "|");
  strcpy(dataGCS, strtokIndx);
  strtokIndx = strtok(NULL, "|");
  Emergency = atoi(strtokIndx);
}

void recTuning() {
  char * strtokIndx;

  strtokIndx = strtok(tempChar, "|");
  strcpy(dataGCS, strtokIndx);

  strtokIndx = strtok(NULL, "|");
  tuning[0] = atof(strtokIndx);//kp

  strtokIndx = strtok(NULL, "|");
  tuning[1] = atof(strtokIndx);//ki

  strtokIndx = strtok(NULL, "|");
  tuning[2] = atof(strtokIndx);//kd

  strtokIndx = strtok(NULL, "|");
  tuning[3] = atof(strtokIndx);//sp
  
}

void tampilan() {
  Serial.print(",");
  Serial.print(Lat, 8);
  Serial.print(",");
  Serial.print(Lon, 8);
  Serial.print(",");
  Serial.print(wpLat, 8);
  Serial.print(",");
  Serial.print(wpLon, 8);
  Serial.print(",");
  Serial.print(heading);
  Serial.print(",");
  Serial.print(bearing);
  Serial.print(",");
  Serial.print(jarak);
  Serial.print(",");
  Serial.print(pwm1);
  Serial.print(",");
  Serial.print(Emergency);
  Serial.print(",");
  Serial.print(tuning[0]);
  Serial.print(",");
  Serial.print(tuning[1]);
  Serial.println("");
}

void trial() {
    Serial.print(Lat, 8);
    Serial.print(",");
    Serial.print(Lon, 8);
    Serial.print(",");
    Serial.print(WP[0], 8);
    Serial.print(",");
    Serial.print(WP[1], 8);
    Serial.print(",");
    Serial.print(WP[2], 8);
    Serial.print(",");
    Serial.print(WP[3], 8);
    Serial.print(",");
    Serial.print(WP[4], 8);
    Serial.print(",");
    Serial.print(WP[5], 8);
    Serial.print(",");
    Serial.print(WP[6], 8);
    Serial.print(",");
    Serial.print(WP[7], 8);
    Serial.print(",");
    Serial.print(WP[8], 8);
    Serial.print(",");
    Serial.print(WP[9], 8);
    Serial.print(",");
    Serial.print(WP[13], 8);
//    Serial.print(",");
//    Serial.print(pwm2);
//    Serial.print(",");
//    Serial.print(rcValue[2]);
    Serial.println("");
}

void kirim() {
  GCS.print("#DAT");
  GCS.print(",");
  GCS.print(Lat, 8);
  GCS.print(",");
  GCS.print(Lon, 8);
  GCS.print(",");
  GCS.print(latB, 8);
  GCS.print(",");
  GCS.print(lonB, 8);
  GCS.print(",");
  GCS.print(heading);
  GCS.print(",");
  GCS.print(bearing);
  GCS.print(",");
  GCS.print(cariError());
  GCS.print(",");
  GCS.print(jarak);
  GCS.print(",");
  GCS.print(pwm1);
  GCS.print(",");
  GCS.print(Start);
  GCS.print(",");
  GCS.print(tuning[0]);
  GCS.print(",");
  GCS.print(tuning[1]);
  GCS.print(",");
  GCS.print(tuning[2]);
  GCS.print(",");
  GCS.print(tuning[3]);
  GCS.print(",");
  GCS.print(Emergency);
  GCS.print(",");
  GCS.print("!");
  GCS.println();
}

void bacaSerial1() {
  char input;
  input = Serial.read();
  switch (input) {
    case 'a':
      wpLat = -7.005145;//Semarang
      wpLon = 110.438126;
      break;
    case 's':
      wpLat = -6.175110;//Jakarta
      wpLon = 106.865036;
      break;
    case 'd':
      wpLat = -6.917464;//Bandung
      wpLon = 107.619125;
      break;
    case 'f':
      wpLat = -7.250445;//Surabaya
      wpLon = 112.768845;
      break;
    case 'g':
      wpLat = -8.100000;//Blitar
      wpLon = 112.150002;
      break;
    case 'h':
      wpLat = -1.265386;//Balikpapan
      wpLon = 116.831200;
      break;
    case 'j':
      wpLat = -5.135399;//Makassar
      wpLon = 119.423790;
      break;
    case 'k':
      wpLat = 21.422487;//Ka'bah
      wpLon = 39.826206;
      break;
    case 'l':
      wpLat = 51.508530;//London
      wpLon = -0.076132;
      break;
  }
}

//void bacaSerial() {
//  baca();
//  if (dataBaru == true) {
//    strcpy(tempChar, dataTerima);
//    parseData();
//    //tampilSerial();
//    dataBaru = false;
//  }
//}
//
//void baca() {
//  static bool terimaData = false;
//  static byte ndx;
//  char header = '<';
//  char endline = '>';
//  char c;
//
//  while (Serial.available() > 0 && dataBaru == false) {
//    c = Serial.read();
//
//    if (terimaData == true) {
//      if (c != endline) {
//        dataTerima[ndx] = c;
//        ndx++;
//        if (ndx >= dataByte) {
//          ndx = dataByte - 1;
//        }
//      }
//      else {
//        dataTerima[ndx] = '\0';
//        terimaData = false;
//        ndx = 0;
//        dataBaru = true;
//      }
//    }
//    else if (c == header) {
//      terimaData = true;
//    }
//  }
//}
//
//void parseData() {
//
//  char * strtokIndx;
//
//  strtokIndx = strtok(tempChar, ",");
//  strcpy(dataPayload, strtokIndx);
//
//  strtokIndx = strtok(NULL, ",");
//  WP[0] = atof(strtokIndx);
//
//  strtokIndx = strtok(NULL, ",");
//  WP[1] = atof(strtokIndx);
//
//  strtokIndx = strtok(NULL, ",");
//  WP[2] = atof(strtokIndx);
//
//  strtokIndx = strtok(NULL, ",");
//  WP[3] = atof(strtokIndx);
//
//  strtokIndx = strtok(NULL, ",");
//  WP[4] = atof(strtokIndx);
//
//  strtokIndx = strtok(NULL, ",");
//  WP[5] = atof(strtokIndx);
//
//  strtokIndx = strtok(NULL, ",");
//  WP[6] = atof(strtokIndx);
//
//  strtokIndx = strtok(NULL, ",");
//  WP[7] = atof(strtokIndx);
//
//  strtokIndx = strtok(NULL, ",");
//  WP[8] = atof(strtokIndx);
//
//  strtokIndx = strtok(NULL, ",");
//  WP[9] = atof(strtokIndx);
//
//}
//
//void bacas() {
//  static bool terimaData = false;
//  static byte ndx;
//  char header = '<';
//  char endline = '>';
//  char c;
//
//  while (Serial.available() > 0 && dataBaru == false) {
//    c = Serial.read();
//
//    if (terimaData == true) {
//      if (c != endline) {
//        dataTerima[ndx] = c;
//        ndx++;
//        if (ndx >= dataByte) {
//          ndx = dataByte - 1;
//        }
//      }
//      else {
//        dataTerima[ndx] = '\0';
//        terimaData = false;
//        ndx = 0;
//        dataBaru = true;
//      }
//    }
//    else if (c == header) {
//      terimaData = true;
//    }
//  }
//
//  if (strstr(dataTerima, "<data")) {
//    char * strtokIndx;
//
//    strtokIndx = strtok(tempChar, ",");
//    strcpy(dataPayload, strtokIndx);
//
//    strtokIndx = strtok(NULL, ",");
//    WP[0] = atof(strtokIndx);
//    Serial.println(WP[0]);
//  }
//}
