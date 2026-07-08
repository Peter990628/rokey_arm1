from rest_framework import serializers
from .models import Medicine, Event, EventItem


class MedicineSerializer(serializers.ModelSerializer):
    class Meta:
        model = Medicine
        fields = "__all__"


class EventItemSerializer(serializers.ModelSerializer):
    medicine_name = serializers.CharField(
        source="medicine.medicine_name",
        read_only=True
    )

    class Meta:
        model = EventItem
        fields = [
            "id",
            "medicine_name",
            "quantity",
            "order",
            "status"
        ]


class EventSerializer(serializers.ModelSerializer):
    items = EventItemSerializer(many=True, read_only=True)

    class Meta:
        model = Event
        fields = [
            "id",
            "prescription_name",
            "status",
            "created_at",
            "items",
        ]


class PrescriptionItemSerializer(serializers.Serializer):
    medicine_name = serializers.CharField()
    quantity = serializers.IntegerField(min_value=1)


class PrescriptionSerializer(serializers.Serializer):
    prescription_name = serializers.CharField(max_length=100)
    items = PrescriptionItemSerializer(many=True)