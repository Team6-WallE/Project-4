from django.shortcuts import render
import requests
from rest_framework import viewsets
from .models import BookTable, BorrowTable, MemberTable
from .serializers import BookSerializer, BorrowSerializer, MemberSerializer


# Create your views here.

class BookView(viewsets.ModelViewSet):
    queryset = BookTable.objects.all()
    serializer_class = BookSerializer

class MemberView(viewsets.ModelViewSet):
    queryset = MemberTable.objects.all()
    serializer_class = MemberSerializer

class BorrowView(viewsets.ModelViewSet):
    queryset = BorrowTable.objects.all()
    serializer_class = BorrowSerializer