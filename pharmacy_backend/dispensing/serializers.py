from rest_framework import serializers
from .models import Medicine, Event, EventItem


class MedicineSerializer(serializers.ModelSerializer):
    medicine_number = serializers.IntegerField(
        source="id",
        read_only=True
    )

    class Meta:
        model = Medicine
        fields = [
            "medicine_number",
            "medicine_name",

            "storage_x",
            "storage_y",
            "storage_z",
            "storage_rx",
            "storage_ry",
            "storage_rz",

            "dispensing_x",
            "dispensing_y",
            "dispensing_z",
            "dispensing_rx",
            "dispensing_ry",
            "dispensing_rz",

            "bottle_tip_offset_x",
            "bottle_tip_offset_y",
            "bottle_tip_offset_z",


            "drawer_x",
            "drawer_y",
            "drawer_z",
            "drawer_rx",
            "drawer_ry",
            "drawer_rz",

            "lid_type",
            "storage_stock",
            "dispensing_stock",
        ]


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