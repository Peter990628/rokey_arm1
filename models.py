from django.db import models


class Medicine(models.Model):
    medicine_name = models.CharField(max_length=100)

    # 약이 보관된 적재소 위치
    storage_location = models.CharField(max_length=20)

    # 약을 리필해야 하는 조제기 위치
    dispensing_location = models.CharField(max_length=20)

    # 뚜껑 타입
    lid_type = models.CharField(max_length=20)

    # 적재소 재고
    storage_stock = models.IntegerField(default=0)

    # 조제기 내부 재고
    dispensing_stock = models.IntegerField(default=0)

    # 조제기 부족 판단 기준
    min_stock = models.IntegerField(default=5)

    def __str__(self):
        return self.medicine_name


class Event(models.Model):
    STATUS_CHOICES = [
        ("WAITING", "WAITING"),
        ("PROCESSING", "PROCESSING"),
        ("DONE", "DONE"),
    ]

    # 처방전에 적힌 이름 / 환자 이름
    prescription_name = models.CharField(
        max_length=100,
        default=""
    )

    status = models.CharField(
        max_length=20,
        choices=STATUS_CHOICES,
        default="WAITING"
    )

    created_at = models.DateTimeField(auto_now_add=True)

    def __str__(self):
        return f"Event {self.id} - {self.prescription_name}"


class EventItem(models.Model):
    event = models.ForeignKey(
        Event,
        on_delete=models.CASCADE,
        related_name="items"
    )

    medicine = models.ForeignKey(
        Medicine,
        on_delete=models.CASCADE
    )

    # 이 처방전에 필요한 약 개수
    quantity = models.IntegerField()

    # 처방 순서
    order = models.IntegerField()

    def __str__(self):
        return f"{self.event.id} - {self.medicine.medicine_name} x {self.quantity}"