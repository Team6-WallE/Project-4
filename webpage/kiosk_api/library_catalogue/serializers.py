from rest_framework import serializers
from .models import BookTable, BorrowTable, MemberTable
from datetime import datetime, timedelta
from django.utils import timezone

class BookSerializer(serializers.HyperlinkedModelSerializer):
    class Meta:
        model = BookTable
        fields = ("book_id", "book_name", "author", "isbn", 'url')
        
class MemberSerializer(serializers.HyperlinkedModelSerializer):
    class Meta:
        model = MemberTable
        fields = ("member_id", "name", "email", 'url')
        
class BorrowSerializer(serializers.HyperlinkedModelSerializer):
    status = serializers.SerializerMethodField()
    class Meta:
        model = BorrowTable
        fields = ("book_id", "member_id", "start_date", "return_date", "status" ,'url')
        
    def get_status(self, obj):
        if obj.start_date <= timezone.now() < obj.return_date:
            return 'Borrowed'
        else:
            return 'Available'
