void PrintT(char* str1, float str2) {
  Serial.print(str1); Serial.print(": "); Serial.print(str2); Serial.print("\t");
}

void PrintN(char* str1, float str2) {
  Serial.print(str1);  Serial.print(": "); Serial.print(str2); Serial.print("\n");
}

void PrintS(char* str1, float str2) {
  Serial.print(str1);  Serial.print(" "); Serial.print(str2); Serial.print(" ");
}
