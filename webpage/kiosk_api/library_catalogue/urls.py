from django.urls import path, include
from . import views
from rest_framework import routers

router = routers.DefaultRouter()
router.register('Books', views.BookView)
router.register('Member', views.MemberView)
router.register('Borrow', views.BorrowView)

urlpatterns = [
    path('', include(router.urls)),
]