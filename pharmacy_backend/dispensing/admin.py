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

        "dispensing_x",
        "dispensing_y",
        "dispensing_z",

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