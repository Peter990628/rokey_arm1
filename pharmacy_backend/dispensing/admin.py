from django.contrib import admin
from .models import Medicine, Event, EventItem


@admin.register(Medicine)
class MedicineAdmin(admin.ModelAdmin):
    list_display = (
        "id",
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
    )


@admin.register(Event)
class EventAdmin(admin.ModelAdmin):
    list_display = (
        "id",
        "prescription_name",
        "status",
        "created_at",
    )


@admin.register(EventItem)
class EventItemAdmin(admin.ModelAdmin):
    list_display = (
        "id",
        "event",
        "medicine",
        "quantity",
        "order",
    )