from django.contrib import admin
from .models import BookTable
from .models import MemberTable
from .models import BorrowTable

# Register your models here.
admin.site.register(BookTable)
admin.site.register(MemberTable)
admin.site.register(BorrowTable)