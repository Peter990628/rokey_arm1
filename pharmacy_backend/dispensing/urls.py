from django.urls import path
from rest_framework.routers import DefaultRouter

from .views import (
    MedicineViewSet,
    EventViewSet,
    PrescriptionCreateAPIView,
    NextTaskAPIView,
    UpdateTaskStatusAPIView,
    RefillMedicineAPIView,
)


router = DefaultRouter()

router.register("medicine", MedicineViewSet)
router.register("events", EventViewSet)

urlpatterns = router.urls + [
    path("prescriptions/", PrescriptionCreateAPIView.as_view()),
    path("tasks/next/", NextTaskAPIView.as_view()),
    path("tasks/status/", UpdateTaskStatusAPIView.as_view()),
    path("tasks/refill/", RefillMedicineAPIView.as_view()),
]