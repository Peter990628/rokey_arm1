from django.db import transaction

from rest_framework import viewsets
from rest_framework.views import APIView
from rest_framework.response import Response

from .models import Medicine, Event, EventItem
from .serializers import (
    MedicineSerializer,
    EventSerializer,
    PrescriptionSerializer,
)


class MedicineViewSet(viewsets.ModelViewSet):
    queryset = Medicine.objects.all().order_by("id")
    serializer_class = MedicineSerializer


class EventViewSet(viewsets.ModelViewSet):
    queryset = Event.objects.all().order_by("-created_at")
    serializer_class = EventSerializer


class PrescriptionCreateAPIView(APIView):
    """
    UI에서 처방전 등록할 때 사용.

    요청 예시:
    {
        "prescription_name": "김철수",
        "items": [
            {
                "medicine_name": "타이레놀",
                "quantity": 2
            }
        ]
    }

    처리:
    - Medicine 존재 여부 확인
    - 조제기 재고 확인
    - Event 생성
    - EventItem 생성
    - dispensing_stock 즉시 감소
    """

    @transaction.atomic
    def post(self, request):
        serializer = PrescriptionSerializer(data=request.data)

        if not serializer.is_valid():
            return Response(serializer.errors, status=400)

        prescription_name = serializer.validated_data["prescription_name"]
        items = serializer.validated_data["items"]

        medicine_list = []

        for item in items:
            medicine_name = item["medicine_name"]
            quantity = item["quantity"]

            try:
                medicine = Medicine.objects.select_for_update().get(
                    medicine_name=medicine_name
                )
            except Medicine.DoesNotExist:
                return Response(
                    {
                        "error": f"{medicine_name} 은(는) 등록되지 않은 약입니다."
                    },
                    status=400
                )

            if medicine.dispensing_stock < quantity:
                return Response(
                    {
                        "error": f"{medicine.medicine_name} 조제기 재고 부족",
                        "current_dispensing_stock": medicine.dispensing_stock,
                        "requested_quantity": quantity,
                    },
                    status=400
                )

            medicine_list.append((medicine, quantity))

        event = Event.objects.create(
            prescription_name=prescription_name,
            status="WAITING"
        )

        for order, (medicine, quantity) in enumerate(medicine_list, start=1):
            EventItem.objects.create(
                event=event,
                medicine=medicine,
                quantity=quantity,
                order=order
            )

            medicine.dispensing_stock -= quantity
            medicine.save()

        return Response(
            {
                "event_id": event.id,
                "prescription_name": event.prescription_name,
                "status": event.status,
                "message": "처방전이 등록되었고 조제기 재고가 차감되었습니다.",
            },
            status=201
        )


class NextTaskAPIView(APIView):
    """
    ROS2 bridge_node가 다음 작업 조회할 때 사용.

    GET /api/tasks/next/
    """

    def get(self, request):
        event = (
            Event.objects
            .filter(status="WAITING")
            .order_by("created_at")
            .first()
        )

        if event is None:
            return Response(
                {
                    "message": "대기 중인 작업이 없습니다."
                },
                status=404
            )

        items = []

        for item in event.items.all().order_by("order"):
            medicine = item.medicine

            items.append({
                "medicine_name": medicine.medicine_name,
                "quantity": item.quantity,
                "order": item.order,
                "storage_location": medicine.storage_location,
                "dispensing_location": medicine.dispensing_location,
                "lid_type": medicine.lid_type,
                "storage_stock": medicine.storage_stock,
                "dispensing_stock": medicine.dispensing_stock,
                "min_stock": medicine.min_stock,
                "refill_needed": medicine.dispensing_stock <= medicine.min_stock,
            })

        return Response(
            {
                "event_id": event.id,
                "prescription_name": event.prescription_name,
                "status": event.status,
                "items": items,
            },
            status=200
        )


class UpdateTaskStatusAPIView(APIView):
    """
    ROS2 또는 프론트에서 작업 상태 변경할 때 사용.

    POST /api/tasks/status/
    {
        "event_id": 1,
        "status": "PROCESSING"
    }
    """

    def post(self, request):
        event_id = request.data.get("event_id")
        status_value = request.data.get("status")

        try:
            event = Event.objects.get(id=event_id)

        except Event.DoesNotExist:
            return Response(
                {
                    "error": "Event가 존재하지 않습니다."
                },
                status=404
            )

        if status_value not in ["WAITING", "PROCESSING", "DONE"]:
            return Response(
                {
                    "error": "잘못된 status 입니다."
                },
                status=400
            )

        event.status = status_value
        event.save()

        return Response(
            {
                "message": "상태 변경 완료",
                "event_id": event.id,
                "status": event.status,
            },
            status=200
        )


class RefillMedicineAPIView(APIView):
    """
    리필 완료 후 재고 갱신.

    POST /api/tasks/refill/
    {
        "medicine_name": "타이레놀",
        "amount": 20
    }

    처리:
    - storage_stock 감소
    - dispensing_stock 증가
    """

    @transaction.atomic
    def post(self, request):
        medicine_name = request.data.get("medicine_name")
        amount = request.data.get("amount")

        if medicine_name is None or amount is None:
            return Response(
                {
                    "error": "medicine_name과 amount가 필요합니다."
                },
                status=400
            )

        try:
            amount = int(amount)
        except ValueError:
            return Response(
                {
                    "error": "amount는 숫자여야 합니다."
                },
                status=400
            )

        if amount <= 0:
            return Response(
                {
                    "error": "amount는 1 이상이어야 합니다."
                },
                status=400
            )

        try:
            medicine = Medicine.objects.select_for_update().get(
                medicine_name=medicine_name
            )
        except Medicine.DoesNotExist:
            return Response(
                {
                    "error": f"{medicine_name} 은(는) 등록되지 않은 약입니다."
                },
                status=404
            )

        if medicine.storage_stock < amount:
            return Response(
                {
                    "error": f"{medicine.medicine_name} 적재소 재고 부족",
                    "current_storage_stock": medicine.storage_stock,
                    "requested_amount": amount,
                },
                status=400
            )

        medicine.storage_stock -= amount
        medicine.dispensing_stock += amount
        medicine.save()

        return Response(
            {
                "message": "리필 완료",
                "medicine_name": medicine.medicine_name,
                "storage_stock": medicine.storage_stock,
                "dispensing_stock": medicine.dispensing_stock,
            },
            status=200
        )