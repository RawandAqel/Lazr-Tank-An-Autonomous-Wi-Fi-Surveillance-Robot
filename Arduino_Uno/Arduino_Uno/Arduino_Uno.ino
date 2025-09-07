const int ldrPin = A0;     // حساس الضوء (LDR) موصول على A0
const int ledPin = 2;      // الضو (LED) موصول على D2

void setup() {
  pinMode(ledPin, OUTPUT);     // تحديد مخرج للـ LED
  Serial.begin(9600);          // للتصحيح والمراقبة على الشاشة التسلسلية
}

void loop() {
  int ldrValue = analogRead(ldrPin);  // قراءة قيمة LDR
  Serial.println(ldrValue);           // طباعة القيمة للمراقبة

  // إذا كانت الإضاءة الخارجية قوية (القيمة أقل من 500 مثلاً)
  if (ldrValue > 300) {
    digitalWrite(ledPin, HIGH); // شغّل الضو
  } else {
    digitalWrite(ledPin, LOW);  // طفي الضو
  }

  delay(100); // تأخير بسيط لثبات القراءة
}
